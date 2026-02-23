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

void ECManager::run() {
  init_ec();
  cyclic_loop();
}

void ECManager::init_ec() {
  int ret;

  memset(&ctx, 0, sizeof(ctx));
  memset(IOMap, 0, sizeof(IOMap));

  RCLCPP_INFO(logger, "Connecting to %s", interface.c_str());
  ret = ecx_init(&ctx, interface.c_str());
  if (ret <= 0) {
    RCLCPP_ERROR(logger, "Couldn't initialize SOEM context");
    std::exit(EXIT_FAILURE);
  }

  RCLCPP_INFO(logger, "Discovering EC Nodes");
  ret = ecx_config_init(&ctx); // also requests PreOP state
  if (ret <= 0) {
    RCLCPP_ERROR(logger, "EC Nodes Discovery failed");
    std::exit(EXIT_FAILURE);
  }

  // All nodes should be in EC_STATE_PRE_OP according to tutorial
  if (ecx_statecheck(&ctx, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4) !=
      EC_STATE_PRE_OP) {
    RCLCPP_ERROR(logger, "Not all nodes in EC_STATE_PRE_OP");
    std::exit(EXIT_FAILURE);
  }

  // TODO: More extensive verification of network
  if (ctx.slavecount != config.slavecount) {
    RCLCPP_ERROR(logger, "Expected %d devices, but discovered %d",
		config.slavecount, ctx.slavecount);
    std::exit(EXIT_FAILURE);
  }

  RCLCPP_INFO(logger, "Mapping IO");
  ret = ecx_config_map_group(&ctx, IOMap, 0); // also requests SafeOP state
  if (ret > IOMAP_SIZE) {
    RCLCPP_ERROR(logger, "Couldn't map IO: Buffer to small");
    std::exit(EXIT_FAILURE);
  }

  // All nodes should be in EC_STATE_SAFE_OP according to tutorial
  if (ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) !=
      EC_STATE_SAFE_OP) {
    RCLCPP_ERROR(logger, "Not all nodes in EC_STATE_SAFE_OP");
    std::exit(EXIT_FAILURE);
  }

  RCLCPP_INFO(logger, "Using %d of %lu bytes in IOMap", ret, sizeof(IOMap));

  expectedWKC = ctx.grouplist[0].outputsWKC * 2 + ctx.grouplist[0].inputsWKC;

  RCLCPP_INFO(logger, "Configuring distributed clock");
  ecx_configdc(&ctx);
}

void ECManager::cyclic_loop() {
  // Still in SAFE_OP, PDO transmission is available
  running_ = true;
  int wkc;
  auto next = std::chrono::steady_clock::now();
  auto period = std::chrono::milliseconds(1);

  std::vector<int32_t> motor_commands(ctx.slavecount, 0);
  std::vector<int32_t> motor_feedback(ctx.slavecount, 0);

  // Configuring Drives
  // Setting ModeOfOperation to CyclicSyncPositionMode
  for (int i = 1; i <= ctx.slavecount; i++) {
      CiA402Motor m{(CiA402_Inputs *)ctx.slavelist[i].inputs,
		    (CiA402_Outputs *)ctx.slavelist[i].outputs};
      m.set_mode_of_operation(
	    CiA402Motor::ModeOfOperation::CyclicSyncPositionMode);
  }

  // Transitioning CiA402 State Machine to OPERATION_ENABLED
  {
    int flag = 1;
    RCLCPP_INFO(logger, "Going to operation_enabled");
    while (flag) {
      flag = 0;
      for (int i = 1; i <= ctx.slavecount; i++) {
	CiA402_Inputs * motor_inputs = (CiA402_Inputs*)ctx.slavelist[i].inputs;
	CiA402_Outputs * motor_outputs = (CiA402_Outputs*)ctx.slavelist[i].outputs;
	CiA402Motor m{motor_inputs, motor_outputs};

	RCLCPP_DEBUG(logger, "State of Motor %d: %s", i, m.state_as_string().c_str());
	if (!m.get_state().has_value()) {
	  flag = 1;
	  RCLCPP_INFO(logger, "Motor %d has no decodable state: 0x%04X", i, ((CiA402_Inputs *)ctx.slavelist[i].inputs)->Statusword);
	} else if (m.get_state().value() != CiA402Motor::State::OPERATION_ENABLED) {
	  flag = 1;
	  if (m.get_state().value() == CiA402Motor::State::FAULT) {
	    RCLCPP_INFO(logger, "Motor %d in fault. Trying to recover...", i);
	    m.to_operation_enabled();
	  } else {
	    m.to_operation_enabled();
	  }
	}
      }
      ecx_send_processdata(&ctx);
      ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
    }
    RCLCPP_INFO(logger, "All motors in operation_enabled");
  }

  // Transition to OPERATIONAL
  uint16 reached_state = transition_ec(EC_STATE_OPERATIONAL);
  if (reached_state != EC_STATE_OPERATIONAL) {
    std::exit(EXIT_FAILURE);
  }
  RCLCPP_INFO(logger, "Entering Cyclic Loop");
  while (running_) {
    next += period;

    ecx_send_processdata(&ctx);
    wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);

    if (wkc != expectedWKC) {
      RCLCPP_ERROR(logger, "Not all nodes responded");
    }

    // Iterate over connected drives
    for (int i = 1; i <= ctx.slavecount; i++) {
      CiA402_Outputs *motor_outputs =
	(CiA402_Outputs *)ctx.slavelist[i].outputs;
      CiA402_Inputs *motor_inputs =
	(CiA402_Inputs *)ctx.slavelist[i].inputs;

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

	RCLCPP_DEBUG(logger, "State of Motor %d: %s", i, m.state_as_string().c_str());
	if (!m.get_state().has_value()) {
	  flag = 1;
	  RCLCPP_WARN(logger, "Motor %d has no decodable state: 0x%04X", i, motor_inputs->Statusword);
	} else if (m.get_state().value() != CiA402Motor::State::OPERATION_ENABLED) {
	  m.to_switch_on_disabled();
	  flag = 1;
	}
      }
      ecx_send_processdata(&ctx);
      ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
      std::this_thread::sleep_until(next);
    }
  }

  /* Go to PRE_OP */
  transition_ec(EC_STATE_PRE_OP);

  /* Go to SAFE_OP */
  transition_ec(EC_STATE_SAFE_OP);

  /* Go to INIT state */
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
  do {
    ecx_send_processdata(&ctx);
    ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
    reached_state = ecx_statecheck(&ctx, 0, state, EC_TIMEOUTSTATE * 4);
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
