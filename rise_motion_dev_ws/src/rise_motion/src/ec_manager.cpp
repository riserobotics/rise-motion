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
#include <rise_motion/config.hpp>


ECManager *ECManager::callback_instance_ = nullptr;

ECManager::ECManager(const std::string interface, int cycle_time)
    : interface(interface), logger(rclcpp::get_logger("ECManager")),
      next(std::chrono::steady_clock::now()),
      period(std::chrono::milliseconds(cycle_time)) {}

int ECManager::init_ec() {
  int ret;

  callback_instance_ = this;

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
  if (ctx.slavecount != rise_motion::config::num_motors) {
    RCLCPP_ERROR(logger, "Expected %d devices, but discovered %d",
                 rise_motion::config::num_motors, ctx.slavecount);
    return EXIT_FAILURE;
  }

  for (int i = 1; i <= ctx.slavecount; i++) {
    ctx.slavelist[i].PO2SOconfig = &ECManager::config_pdo_mapping_callback; 
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

  // receive valid PDO data to update PositionValue to current position
  const auto timeout = std::chrono::seconds(1);
  const auto start = std::chrono::steady_clock::now();
  int wkc = 0;
  do {
    ecx_send_processdata(&ctx);
    wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);

    if (std::chrono::steady_clock::now() - start > timeout) 
    {
      RCLCPP_ERROR(logger, "Couldn't receive valid PDO data in time");
      shutdown();
      return;
    }

    std::this_thread::sleep_for(period);
  }
  while (wkc <= 0);

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

  // Transition to OPERATIONAL
  // Ethercat needs to be operational before CiA402 is OPERATION_ENABLED
  uint16 reached_state = transition_ec(EC_STATE_OPERATIONAL);
  if (reached_state != EC_STATE_OPERATIONAL) {
    shutdown();
    return;
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

    // Sleep until next cycle (maintains 1kHz frequency)
    std::this_thread::sleep_until(next);
  }

  shutdown();
  return;
}

void ECManager::shutdown() {
  RCLCPP_INFO(logger, "Shutting down");
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

bool ECManager::sdo_read(uint16 device_id, uint16 index, uint8 subindex,
                         std::vector<uint8> &value, uint8 value_size) {
  int psize = value_size;
  uint8 *buf = new uint8[psize];

  boolean CA = FALSE;
  int wkc = ecx_SDOread(&ctx, device_id, index, subindex, CA, &psize, (void *)buf, EC_TIMEOUTRXM);

  if (wkc <= 0) {
    RCLCPP_ERROR(logger, "SDO read failed: device_id=%d object=0x%04x:%d", device_id, index, subindex);

    return false;
  }

  value.clear();
  for (int i = 0; i < psize; i++) {
    value.push_back(buf[i]);
  }
  
  return true;
}

bool ECManager::sdo_write(uint16 device_id, uint16 index, uint8 subindex,
                          std::vector<uint8> &value) {
  if (value.empty()){
    RCLCPP_ERROR(logger, "Tried to write empty SDO value: device_id=%d object=0x%04x:%d", device_id, index, subindex);

    return false;
  }

  int psize = value.size();
  uint8 *buf = &value[0];

  boolean CA = FALSE;
  int wkc = ecx_SDOwrite(&ctx, device_id, index, subindex, CA, psize, (void *)buf, EC_TIMEOUTRXM);
  
  if (wkc <= 0) {
    RCLCPP_ERROR(logger, "SDO write failed: device_id=%d object=0x%04x:%d", device_id, index, subindex);

    return false;
  }

  return true;
}

bool ECManager::check_sdo_value(uint16 device_id, uint16 index, uint8 subindex, const std::vector<uint8> &expected) {

  std::vector<uint8> actual;

  if (!sdo_read(device_id, index, subindex, actual, expected.size())) {
    return false;
  }

  return actual == expected;
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

bool ECManager::set_pdo_map(uint16 device_id, uint16 map_index, const std::vector<pdoMap::PDOMappingEntry> &entries) {

  if (entries.size() > UINT8_MAX) {
    RCLCPP_ERROR(logger, "Too many entries for PDO map 0x%04x", map_index);

    return false;
  }

  auto count = pdoMap::u8(0);

  // disable PDO map
  if (!sdo_write(device_id, map_index, 0, count)) {
    return false;
  }

  // set mapping
  for (size_t i = 0; i < entries.size(); ++i) {
    auto entry = entries[i];

    if (!sdo_write(device_id, map_index, static_cast<uint8>(i + 1), entry)) {
      return false;
    }
  }

  // enable number of defined entries
  count = pdoMap::u8(entries.size());

  if (!sdo_write(device_id, map_index, 0, count)) {
    return false;
  }

  // check number of enabled entries
  if (!check_sdo_value(device_id, map_index, 0, count)) {
    return false;
  }

  // check entries
  for (size_t i = 0; i < entries.size(); ++i) {
    if (!check_sdo_value(device_id, map_index, static_cast<uint8>(i + 1), entries[i])) {
      return false;
    }
  }

  return true;
}

bool ECManager::config_pdo_mapping(uint16 device_id) {
  RCLCPP_INFO(logger, "Configuring PDO mapping for device_id %d", device_id);

  // disable current PDO maps
  auto disabled = pdoMap::u8(0);

  if (!sdo_write(device_id, 0x1C12, 0, disabled) || !sdo_write(device_id, 0x1C13, 0, disabled)) {
    RCLCPP_ERROR(logger, "Couldn't disable PDO assignments for device_id %d", device_id);
    return false;
  }

  // set RxPDO: master -> drive
  if (!set_pdo_map(device_id, 0x1600, pdoMap::RX_PDO_1600) || !set_pdo_map(device_id, 0x1601, pdoMap::RX_PDO_1601) ||
      !set_pdo_map(device_id, 0x1602, pdoMap::RX_PDO_1602)) {
    RCLCPP_ERROR(logger, "Couldn't configure RxPDOs for device_id %d", device_id);
    return false;
  }

  // set TxPDO: drive -> master
  if (!set_pdo_map(device_id, 0x1A00, pdoMap::TX_PDO_1A00) || !set_pdo_map(device_id, 0x1A01, pdoMap::TX_PDO_1A01) ||
      !set_pdo_map(device_id, 0x1A02, pdoMap::TX_PDO_1A02) || !set_pdo_map(device_id, 0x1A03, pdoMap::TX_PDO_1A03)) {
    RCLCPP_ERROR(logger, "Couldn't configure TxPDOs for device_id %d", device_id);
    return false;
  }

  // enable PDO maps
  if (!set_pdo_map(device_id, 0x1C12, pdoMap::RX_ASSIGNMENT) || !set_pdo_map(device_id, 0x1C13, pdoMap::TX_ASSIGNMENT)) {
    RCLCPP_ERROR(logger, "Couldn't configure PDO assignments for device_id %d", device_id);
    return false;
  }

  RCLCPP_INFO(logger, "PDO mapping configured for drive %d", device_id);

  return true;
}

int ECManager::config_pdo_mapping_callback(ecx_contextt *ctx, uint16 device_id) {
  if (callback_instance_ == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("ECManager"), "config_pdo_mapping_callback called but callback_instance_ is null");
    return 0;
  }

  return callback_instance_->config_pdo_mapping(device_id) ? 1 : 0;
}
