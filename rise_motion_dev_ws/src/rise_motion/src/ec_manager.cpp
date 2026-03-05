#include <chrono>
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

ECManager::ECManager(const std::string interface) : interface(interface) {}

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
  return EXIT_SUCCESS;
}

void ECManager::cyclic_loop() {
  // Still in SAFE_OP, PDO transmission is available
  running_ = true;
  int wkc;
  auto next = std::chrono::steady_clock::now();
  auto period = std::chrono::milliseconds(1);

  std::vector<int32_t> motor_commands(ctx.slavecount, 0);
  std::vector<int32_t> motor_feedback(ctx.slavecount, 0);

  // Transition to OPERATIONAL
  // Ethercat needs to be operational before CiA402 is OPERATION_ENABLED
  uint16 reached_state = transition_ec(EC_STATE_OPERATIONAL);
  if (reached_state != EC_STATE_OPERATIONAL) {
    std::exit(EXIT_FAILURE);
  }

  // Configuring Drives
  for (int i = 1; i <= ctx.slavecount; i++) {
    CiA402_Outputs *motor_outputs =
      (CiA402_Outputs *)ctx.slavelist[i].outputs;
    CiA402_Inputs *motor_inputs =
      (CiA402_Inputs *)ctx.slavelist[i].inputs;
    CiA402Motor m{motor_inputs, motor_outputs};
    // Setting ModeOfOperation to CyclicSyncPositionMode
    m.set_mode_of_operation(CiA402Motor::ModeOfOperation::CyclicSyncPositionMode);
    // Set Position to Current Position
    motor_commands[i-1] = motor_inputs->PositionValue;
    motor_outputs->TargetPosition = motor_inputs->PositionValue;
    RCLCPP_INFO(logger, "Configured Motor %d: (%d)", i, motor_inputs->PositionValue);
  }

  // Transitioning CiA402 State Machine to OPERATION_ENABLED
  {
    int flag = 1;
    RCLCPP_INFO(logger, "Going to operation_enabled");
    while (flag) {
      next += period;
      flag = 0;
      for (int i = 1; i <= ctx.slavecount; i++) {
	CiA402_Inputs * motor_inputs = (CiA402_Inputs*)ctx.slavelist[i].inputs;
	CiA402_Outputs * motor_outputs = (CiA402_Outputs*)ctx.slavelist[i].outputs;
	CiA402Motor m{motor_inputs, motor_outputs};

	RCLCPP_INFO(logger, "State of Motor %d: %s", i, m.state_as_string().c_str());
	if (!m.get_state().has_value()) {
	  flag = 1;
	  RCLCPP_WARN(logger, "Motor %d has no decodable state: 0x%04X", i, ((CiA402_Inputs *)ctx.slavelist[i].inputs)->Statusword);
	} else if (m.get_state().value() != CiA402Motor::State::OPERATION_ENABLED) {
	  flag = 1;
	  if (m.get_state().value() == CiA402Motor::State::FAULT) {
	    RCLCPP_ERROR(logger, "Motor %d in fault. Exiting...", i);
	    exit(EXIT_FAILURE);
//	    RCLCPP_WARN(logger, "Motor %d in fault. Trying to recover...", i);
//	    m.to_operation_enabled();
	  } else {
	    m.to_operation_enabled();
	  }
	}
      }
      ecx_send_processdata(&ctx);
      ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
      std::this_thread::sleep_until(next);
    }
    RCLCPP_INFO(logger, "All motors in operation_enabled");
  }

  RCLCPP_INFO(logger, "Entering Cyclic Loop");
  while (running_) {
    next += period;

    ecx_send_processdata(&ctx);
    wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
    ecx_mbxhandler(&ctx, 0, 4);

    if (wkc != expectedWKC) {
      RCLCPP_ERROR(logger, "Not all nodes responded");
    }

    // Iterate over connected drives
    for (int i = 1; i <= ctx.slavecount; i++) {
      CiA402_Outputs *motor_outputs =
	(CiA402_Outputs *)ctx.slavelist[i].outputs;
      CiA402_Inputs *motor_inputs =
	(CiA402_Inputs *)ctx.slavelist[i].inputs;
      CiA402Motor m{motor_inputs, motor_outputs};

      if (!m.get_state().has_value()) {
	RCLCPP_ERROR(logger, "Motor %d has no state", i);
      } else if (m.get_state().value() != CiA402Motor::State::OPERATION_ENABLED) {
	RCLCPP_ERROR(logger, "Motor %d is not in OPERATION_ENABLED", i);
      }

      if (cmd_apsa.perf_read(motor_commands)) {
	// New commands received! Apply them to EtherCAT nodes
	motor_outputs->TargetPosition = motor_commands[i-1];
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
		     motor_outputs->Controlword, motor_outputs->OpMode,
		     motor_outputs->TargetTorque, motor_outputs->TargetPosition,
		     motor_outputs->TargetVelocity, motor_outputs->TorqueOffset,
		     motor_outputs->TuningCommand, motor_outputs->PhysicalOutputs,
		     motor_outputs->BitMask, motor_outputs->UserMOSI,
		     motor_outputs->VelocityOffset);
      }
      motor_outputs->TargetPosition = motor_commands[i-1];
      motor_feedback[i-1] = motor_inputs->PositionValue;
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
		   motor_inputs->Statusword,
		   motor_inputs->OpModeDisplay,
		   motor_inputs->PositionValue,
		   motor_inputs->VelocityValue,
		   motor_inputs->TorqueValue,
		   motor_inputs->AnalogInput1,
		   motor_inputs->AnalogInput2,
		   motor_inputs->AnalogInput3,
		   motor_inputs->AnalogInput4,
		   motor_inputs->TuningStatus,
		   motor_inputs->DigitalInputs,
		   motor_inputs->UserMISO,
		   motor_inputs->Timestamp,
		   motor_inputs->PositionDemandInternalValue,
		   motor_inputs->VelocityDemandValue,
		   motor_inputs->TorqueDemand
		   );
    }

    // Make feedback available to ROS publisher (wait-free)
    feedback_apsa.perf_write(motor_feedback);

    // Sleep until next cycle (maintains 1kHz frequency)
    std::this_thread::sleep_until(next);
  }

  RCLCPP_INFO(logger, "Exiting cyclic loop");

  // Transitioning Motors to SWITCH_ON_DISABLED
  {
    int flag = 1;
    while (flag) {
      next += period;
      flag = 0;
      for (int i = 1; i <= ctx.slavecount; i++) {
	CiA402_Inputs * motor_inputs = (CiA402_Inputs*)ctx.slavelist[i].inputs;
	CiA402_Outputs * motor_outputs = (CiA402_Outputs*)ctx.slavelist[i].outputs;
	CiA402Motor m{motor_inputs, motor_outputs};

	RCLCPP_INFO(logger, "State of Motor %d: %s", i, m.state_as_string().c_str());
	if (!m.get_state().has_value()) {
	  flag = 1;
	  RCLCPP_WARN(logger, "Motor %d has no decodable state: 0x%04X", i, motor_inputs->Statusword);
	} else if (m.get_state().value() != CiA402Motor::State::SWITCH_ON_DISABLED) {
	  m.to_switch_on_disabled();
	  flag = 1;
	}
      }
      ecx_send_processdata(&ctx);
      ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
      std::this_thread::sleep_until(next);
    }
  }

  transition_ec(EC_STATE_PRE_OP);
  transition_ec(EC_STATE_SAFE_OP);
  transition_ec(EC_STATE_INIT);

  ecx_close(&ctx);
}

void ECManager::stop() { running_ = false; }

bool ECManager::get_motor_values_apsa(std::vector<int32_t> &motor_values) {
  // comm_read() returns true if new data is available, false otherwise
  return feedback_apsa.comm_read(motor_values);
}

bool ECManager::set_motor_values_apsa(
    const std::vector<int32_t> &motor_values) {
  // comm_write() queues the data for the EtherCAT loop to pick up
  return cmd_apsa.comm_write(motor_values);
}

rclcpp::Logger ECManager::logger = rclcpp::get_logger("ECManager");

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
  auto next = std::chrono::steady_clock::now();
  auto period = std::chrono::milliseconds(1);
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
    for (int i = 1; i <= ctx.slavecount; i++)
      {
	if (ctx.slavelist[i].state != state)
	  {
	    RCLCPP_WARN(logger, "Node %d State=%2x StatusCode=%4x : %s",
			i, ctx.slavelist[i].state, ctx.slavelist[i].ALstatuscode, ec_ALstatuscode2string(ctx.slavelist[i].ALstatuscode));
	  }
      }
  }
  return reached_state;
}

bool ECManager::sdo_read(uint16 device_id, uint16 index, uint8 subindex,
                         std::vector<uint8> &value) {
  int psize = 64;
  uint8 *buf = new uint8[psize];

  boolean CA = FALSE;
  int wkc = ecx_SDOread(&ctx, device_id, index, subindex, CA, &psize,
                        (void *)buf, EC_TIMEOUTRXM);

  value.clear();
  for (int i = 0; i < psize; i++) {
    value.push_back(buf[i]);
  }
  RCLCPP_INFO(logger, "%d:%d", wkc, expectedWKC);
  //  return (wkc == expectedWKC);
  return true;
}

bool ECManager::sdo_write(uint16 device_id, uint16 index, uint8 subindex,
                          std::vector<uint8> &value) {
  int psize = value.size();
  uint8 *buf = new uint8[psize];

  boolean CA = FALSE;
  int wkc = ecx_SDOwrite(&ctx, device_id, index, subindex, CA, psize,
                         (void *)buf, EC_TIMEOUTRXM);
  RCLCPP_INFO(logger, "%d:%d", wkc, expectedWKC);
  //  return (wkc == expectedWKC);
  return true;
}

