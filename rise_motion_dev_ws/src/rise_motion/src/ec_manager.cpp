#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rise_motion/cia402.hpp>
#include <rise_motion/ec_manager.hpp>
#include <soem/soem.h>

// expected config, needs to be retrieved from config node
struct {
  int slavecount = 1;
} config;

ECManager::ECManager(const std::string interface, int cycle_time)
    : interface(interface), logger(rclcpp::get_logger("ECManager")),
      next(std::chrono::steady_clock::now()),
      period(std::chrono::milliseconds(cycle_time)) {}

int ECManager::init_ec() {
  int ret;

  memset(&ctx, 0, sizeof(ctx));
  memset(IOMap, 0, sizeof(IOMap));

  RCLCPP_INFO(logger, "Connecting to %s", interface.c_str());
  ret = ecx_init(&ctx, interface.c_str());
  if (ret <= 0) {
    RCLCPP_ERROR(logger, "Couldn't initialize SOEM context");
    return EXIT_FAILURE;
  }

  RCLCPP_INFO(logger, "Discovering EC Nodes");
  ret = ecx_config_init(&ctx); // also requests PreOP state
  if (ret <= 0) {
    RCLCPP_ERROR(logger, "EC Nodes Discovery failed");
    return EXIT_FAILURE;
  }

  // All nodes should be in EC_STATE_PRE_OP according to tutorial
  if (ecx_statecheck(&ctx, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4) !=
      EC_STATE_PRE_OP) {
    RCLCPP_ERROR(logger, "Not all nodes in EC_STATE_PRE_OP");
    return EXIT_FAILURE;
  }

  // TODO: More extensive verification of network
  if (ctx.slavecount != config.slavecount) {
    RCLCPP_ERROR(logger, "Expected %d devices, but discovered %d",
                 config.slavecount, ctx.slavecount);
    return EXIT_FAILURE;
  }

  RCLCPP_INFO(logger, "Mapping IO");
  ret = ecx_config_map_group(&ctx, IOMap, 0); // also requests SafeOP state
  if (ret > IOMAP_SIZE) {
    RCLCPP_ERROR(logger, "Couldn't map IO: Buffer to small");
    return EXIT_FAILURE;
  }

  // All nodes should be in EC_STATE_SAFE_OP according to tutorial
  if (ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) !=
      EC_STATE_SAFE_OP) {
    RCLCPP_ERROR(logger, "Not all nodes in EC_STATE_SAFE_OP");
    return EXIT_FAILURE;
  }

  RCLCPP_INFO(logger, "Using %d of %lu bytes in IOMap", ret, sizeof(IOMap));

  // Enable mailboxes for SDO
  for (int i = 1; i <= ctx.slavecount; i++) {
    if (ctx.slavelist[i].CoEdetails > 0) {
      ecx_slavembxcyclic(&ctx, i);
      RCLCPP_INFO(logger, "Enabled mailbox for drive %d", i);
    }
  }

  expectedWKC = ctx.grouplist[0].outputsWKC * 2 + ctx.grouplist[0].inputsWKC;

  RCLCPP_INFO(logger, "Configuring distributed clock");
  ecx_configdc(&ctx);

  // Create motor classes
  motors.clear();
  for (int i = 1; i <= ctx.slavecount; i++) {
    CiA402_Outputs *motor_outputs = (CiA402_Outputs *)ctx.slavelist[i].outputs;
    CiA402_Inputs *motor_inputs = (CiA402_Inputs *)ctx.slavelist[i].inputs;
    CiA402Motor m{motor_inputs, motor_outputs};
    motors.push_back(m);
  }
  return EXIT_SUCCESS;
}

void ECManager::cyclic_loop() {
  // Still in SAFE_OP, PDO transmission is available
  next = std::chrono::steady_clock::now();
  running_ = true;

  std::vector<int32_t> motor_commands(ctx.slavecount, 0);
  std::vector<int32_t> motor_feedback(ctx.slavecount, 0);

  // Transition to OPERATIONAL
  // Ethercat needs to be operational before CiA402 is OPERATION_ENABLED
  uint16 reached_state = transition_ec(EC_STATE_OPERATIONAL);
  if (reached_state != EC_STATE_OPERATIONAL) {
    shutdown();
    return;
  }

  // Configuring Drives
  for (size_t i = 0; i < motors.size(); i++) {
    CiA402Motor &m = motors[i];
    // Setting ModeOfOperation to CyclicSyncPositionMode
    m.set_mode_of_operation(
        CiA402Motor::ModeOfOperation::CyclicSyncPositionMode);

    // Set Position to Current Position
    motor_commands[i] = m.inputs->PositionValue;
    m.outputs->TargetPosition = m.inputs->PositionValue;
    RCLCPP_INFO(logger, "Configured Motor %zu: Init Position(%d)", i + 1,
                m.inputs->PositionValue);
  }

  // Transitioning CiA402 State Machine to OPERATION_ENABLED
  if (!transition_motors_to(CiA402Motor::State::OPERATION_ENABLED)) {
    RCLCPP_ERROR(logger, "Couldn't transition all motors to OPERATION_ENABLED");
    shutdown();
    return;
  }

  RCLCPP_INFO(logger, "All motors in operation_enabled");

  RCLCPP_INFO(logger, "Entering Cyclic Loop");
  next = std::chrono::steady_clock::now();
  while (running_) {
    int wkc;
    next += period;

    ecx_send_processdata(&ctx);
    wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
    ecx_mbxhandler(&ctx, 0, 4);

    if (wkc != expectedWKC) {
      RCLCPP_ERROR(logger, "Not all nodes responded");
      shutdown();
      return;
    }

    // Iterate over connected drives
    for (size_t i = 0; i < motors.size(); i++) {
      CiA402Motor &m = motors[i];

      if (!m.get_state().has_value()) {
        RCLCPP_ERROR(logger, "Motor %zu has no state", i + 1);
        shutdown();
        return;
      } else if (m.get_state().value() !=
                 CiA402Motor::State::OPERATION_ENABLED) {
        RCLCPP_ERROR(logger, "Motor %zu is not in OPERATION_ENABLED", i + 1);
        shutdown();
        return;
      }

      // Try to get new data
      cmd_apsa.perf_read(motor_commands);
      m.outputs->TargetPosition = motor_commands[i];
      motor_feedback[i] = m.inputs->PositionValue;

      RCLCPP_DEBUG(logger,
                   "Motor Outputs:\n"
                   "\tControlword: 0x%04X\n"
                   "\tOpMode: %d\n"
                   "\tTargetTorque: %d\n"
                   "\tTargetPosition: %d\n"
                   "\tTargetVelocity: %d\n"
                   "\tTorqueOffset: %d\n"
                   "\tTuningCommand: %d\n"
                   "\tPhysicalOutputs: %d\n"
                   "\tBitMask: 0x%08X\n"
                   "\tUserMOSI: 0x%08X\n"
                   "\tVelocityOffset: %d\n",
                   m.outputs->Controlword, m.outputs->OpMode,
                   m.outputs->TargetTorque, m.outputs->TargetPosition,
                   m.outputs->TargetVelocity, m.outputs->TorqueOffset,
                   m.outputs->TuningCommand, m.outputs->PhysicalOutputs,
                   m.outputs->BitMask, m.outputs->UserMOSI,
                   m.outputs->VelocityOffset);
      RCLCPP_DEBUG(logger,
                   "Motor Inputs:\n"
                   "\tStatusword: 0x%04X\n"
                   "\tOpModeDisplay: %d\n"
                   "\tPositionValue: %d\n"
                   "\tVelocityValue: %d\n"
                   "\tTorqueValue: %d\n"
                   "\tAnalogInput1: %u\n"
                   "\tAnalogInput2: %u\n"
                   "\tAnalogInput3: %u\n"
                   "\tAnalogInput4: %u\n"
                   "\tTuningStatus: 0x%08X\n"
                   "\tDigitalInputs: 0x%08X\n"
                   "\tUserMISO: 0x%08X\n"
                   "\tTimestamp: %u\n"
                   "\tPositionDemandInternalValue: %d\n"
                   "\tVelocityDemandValue: %d\n"
                   "\tTorqueDemand: %d\n",
                   m.inputs->Statusword, m.inputs->OpModeDisplay,
                   m.inputs->PositionValue, m.inputs->VelocityValue,
                   m.inputs->TorqueValue, m.inputs->AnalogInput1,
                   m.inputs->AnalogInput2, m.inputs->AnalogInput3,
                   m.inputs->AnalogInput4, m.inputs->TuningStatus,
                   m.inputs->DigitalInputs, m.inputs->UserMISO,
                   m.inputs->Timestamp, m.inputs->PositionDemandInternalValue,
                   m.inputs->VelocityDemandValue, m.inputs->TorqueDemand);
    }

    // Make feedback available to ROS publisher (wait-free)
    feedback_apsa.perf_write(motor_feedback);

    process_sdo_request();

    // Sleep until next cycle (maintains 1kHz frequency)
    std::this_thread::sleep_until(next);
  }

  shutdown();
  return;
}

void ECManager::process_sdo_request()
{
  constexpr auto MINIMUM_SDO_TIMEOUT = std::chrono::microseconds{200};

  const auto now = std::chrono::steady_clock::now();

  if (now >= next){
    return;
  }

  const auto time_left = std::chrono::duration_cast<std::chrono::microseconds>(next - now);

  if (time_left <= MINIMUM_SDO_TIMEOUT){
    return;
  }

  auto job = sdo_scheduler_.get_job();

  if (!job.has_value()) {
    return;
  }

  const int timeout_us = static_cast<int>((time_left).count());

  SdoScheduler::AttemptResult attempt_result;

  if (job->request.operation == SdoScheduler::Operation::READ) {
    std::vector<uint8> value;

    const bool success = sdo_read(
      job->request.device_id, job->request.index, job->request.subindex, value, job->request.read_size, timeout_us);

    attempt_result = {
      success ? SdoScheduler::AttemptStatus::SUCCESS : SdoScheduler::AttemptStatus::RETRYABLE_FAILURE, std::move(value)};

  } 
  else {
    const bool success = sdo_write(
      job->request.device_id, job->request.index, job->request.subindex, job->request.write_value, timeout_us);

    attempt_result = {success ? SdoScheduler::AttemptStatus::SUCCESS : SdoScheduler::AttemptStatus::RETRYABLE_FAILURE, {}};
  }

  sdo_scheduler_.complete_attempt(job->id, std::move(attempt_result));
}

void ECManager::shutdown() {
  RCLCPP_INFO(logger, "Shutting down");

  sdo_scheduler_.cancel_all();

  if (!transition_motors_to(CiA402Motor::State::SWITCH_ON_DISABLED)) {
    RCLCPP_ERROR(logger,
                 "Couldn't transition all motors to SWITCH_ON_DISABLED");
  }

  transition_ec(EC_STATE_PRE_OP);
  transition_ec(EC_STATE_SAFE_OP);
  transition_ec(EC_STATE_INIT);

  ecx_close(&ctx);
  running_ = false;
}

void ECManager::stop() { running_ = false; }

bool ECManager::is_running() { return running_; }

bool ECManager::get_motor_values_apsa(std::vector<int32_t> &motor_values) {
  // comm_read() returns true if new data is available, false otherwise
  return feedback_apsa.comm_read(motor_values);
}

bool ECManager::set_motor_values_apsa(
    const std::vector<int32_t> &motor_values) {
  // comm_write() queues the data for the EtherCAT loop to pick up
  return cmd_apsa.comm_write(motor_values);
}

uint16 ECManager::transition_ec(uint16 state) {
  // Get state_string
  std::string state_string = "Unknown";
  {
    switch (state) {
    case EC_STATE_INIT:
      state_string = "EC_STATE_INIT";
      break;
    case EC_STATE_PRE_OP:
      state_string = "EC_STATE_PRE_OP";
      break;
    case EC_STATE_SAFE_OP:
      state_string = "EC_STATE_SAFE_OP";
      break;
    case EC_STATE_OPERATIONAL:
      state_string = "EC_STATE_OPERATIONAL";
      break;
    }
  }

  RCLCPP_INFO(logger, "Transition Ethercat State to %s", state_string.c_str());

  ctx.slavelist[0].state = state;
  ecx_writestate(&ctx, 0);
  int chk = 200;
  uint16 reached_state;
  next = std::chrono::steady_clock::now();
  do {
    next += period;
    ecx_send_processdata(&ctx);
    ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
    reached_state = ecx_statecheck(&ctx, 0, state, EC_TIMEOUTSTATE * 4);
    std::this_thread::sleep_until(next);
  } while (chk-- && (ctx.slavelist[0].state != state));
  if (reached_state != state) {
    RCLCPP_WARN(logger, "Couldn't transition to %s", state_string.c_str());
    ecx_readstate(&ctx);
    for (int i = 1; i <= ctx.slavecount; i++) {
      if (ctx.slavelist[i].state != state) {
        RCLCPP_WARN(logger, "Node %d State=%2x StatusCode=%4x : %s", i,
                    ctx.slavelist[i].state, ctx.slavelist[i].ALstatuscode,
                    ec_ALstatuscode2string(ctx.slavelist[i].ALstatuscode));
      }
    }
  }
  return reached_state;
}

bool ECManager::cancel_sdo_request(SdoScheduler::JobID id)
{
  return sdo_scheduler_.cancel(id);
}

SdoScheduler::Submission ECManager::enqueue_sdo_read(
  uint16 device_id, uint16 index, uint8 subindex, uint8 value_size, SdoScheduler::RetryOptions retry_options)
{
  return sdo_scheduler_.enqueue_read(device_id, index, subindex, value_size, retry_options);
}

SdoScheduler::Submission ECManager::enqueue_sdo_write(
  uint16 device_id, uint16 index, uint8 subindex, std::vector<uint8> value, SdoScheduler::RetryOptions retry_options)
{
  return sdo_scheduler_.enqueue_write(device_id, index, subindex, value, retry_options);
}

bool ECManager::sdo_read(uint16 device_id, uint16 index, uint8 subindex,
                         std::vector<uint8> &value, uint16 value_size, int timeout_us) 
{
  value.resize(value_size);
  int psize = value_size;

  boolean CA = FALSE;
  int wkc = ecx_SDOread(&ctx, device_id, index, subindex, CA, &psize,
                        value.data(), timeout_us);

  if (wkc <= 0) {
    value.clear();
    return false;
  }

  RCLCPP_INFO(logger, "%d:%d", wkc, expectedWKC);
  //  return (wkc == expectedWKC);
  return true;
}

bool ECManager::sdo_write(uint16 device_id, uint16 index, uint8 subindex,
                          std::vector<uint8> &value, int timeout_us) 
{
  int psize = value.size();

  boolean CA = FALSE;
  int wkc = ecx_SDOwrite(&ctx, device_id, index, subindex, CA, psize,
                         value.data(), timeout_us);

  if (wkc <= 0) {
    return false;
  }

  RCLCPP_INFO(logger, "%d:%d", wkc, expectedWKC);
  //  return (wkc == expectedWKC);
  return true;
}

bool ECManager::transition_motors_to(CiA402Motor::State state) {
  int tries_left = 1000;
  int continue_flag = 1;
  next = std::chrono::steady_clock::now();
  while (continue_flag && tries_left > 0) {
    next += period;
    tries_left--;
    continue_flag = 0;
    for (size_t i = 0; i < motors.size(); i++) {
      CiA402Motor &m = motors[i];
      RCLCPP_DEBUG(logger, "State of Motor %zu: %s", i + 1,
                   m.state_as_string().c_str());
      if (!m.get_state().has_value()) {
        continue_flag = 1;
        RCLCPP_WARN(logger, "Motor %zu has no decodable state: 0x%04X", i + 1,
                    m.inputs->Statusword);
      } else if (m.get_state().value() == CiA402Motor::State::FAULT) {
        RCLCPP_ERROR(logger, "Motor %zu in fault", i + 1);
        return false;
      } else if (m.get_state().value() != state) {
        m.transition_to(state);
        continue_flag = 1;
      }
    }
    ecx_send_processdata(&ctx);
    ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
    std::this_thread::sleep_until(next);
  }
  if (continue_flag) {
    RCLCPP_ERROR(logger, "Couldn't transition motors in time");
    return false;
  }
  return true;
}
