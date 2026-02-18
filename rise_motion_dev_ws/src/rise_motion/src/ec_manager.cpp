#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rise_motion/cia402.hpp>
#include <rise_motion/ec_manager.hpp>
#include <soem/soem.h>
#include <string>
#include <thread>
#include <vector>

// expected config, needs to be retrieved from config node
struct {
  int slavecount = 1;
} config;

ECManager::ECManager(const std::string interface) : interface(interface) {}

void ECManager::init_ec() {
  int ret;

  memset(&ctx, 0, sizeof(ctx));
  memset(IOMap, 0, sizeof(IOMap));

  ctx.packedMode = TRUE; // Not sure if necessary

  RCLCPP_INFO(logger, "Connecting to %s", interface.c_str());
  ret = ecx_init(&ctx, interface.c_str());
  if (ret <= 0) {
    RCLCPP_WARN(logger, "Couldn't initialize SOEM context");
    std::exit(EXIT_FAILURE);
  }

  RCLCPP_INFO(logger, "Discovering EC Nodes");
  ret = ecx_config_init(&ctx);
  if (ret <= 0) {
    RCLCPP_WARN(logger, "EC Nodes Discovery failed");
    std::exit(EXIT_FAILURE);
  }

  if (ctx.slavecount != config.slavecount) {
    RCLCPP_WARN(logger, "Expected %d devices, but discovered %d",
		config.slavecount, ctx.slavecount);
    std::exit(EXIT_FAILURE);
  }

  transition_ec(EC_STATE_PRE_OP);

  RCLCPP_INFO(logger, "Mapping IO");
  ret = ecx_config_map_group(&ctx, IOMap, 0);
  if (ret > IOMAP_SIZE) {
    RCLCPP_WARN(logger, "Couldn't map IO: Buffer to small");
    std::exit(EXIT_FAILURE);
  }
  RCLCPP_INFO(logger, "Using %d of %lu bytes in IOMap", ret, sizeof(IOMap));

  expectedWKC = ctx.grouplist[0].outputsWKC * 2 + ctx.grouplist[0].inputsWKC;

  RCLCPP_INFO(logger, "Configuring distributed clock");
  ecx_configdc(&ctx);

  transition_ec(EC_STATE_SAFE_OP);
}

void ECManager::transition_to_operational() {
  // Transitions Ethercat State Machine to operational
  RCLCPP_INFO(logger, "Transitioning to operational mode");
  ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
  ecx_writestate(&ctx, 0);

  // check if nodes entered operational mode
  int chk = 200;
  do {
    ecx_send_processdata(&ctx);
    ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
    ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, 50000);
  } while (chk-- && (ctx.slavelist[0].state != EC_STATE_OPERATIONAL));
  if (ctx.slavelist[0].state != EC_STATE_OPERATIONAL) {
    RCLCPP_WARN(logger, "Couldn't transition to operational");
    std::exit(EXIT_FAILURE);
  }
}
void ECManager::cyclic_loop() {
  transition_to_operational();
  running_ = true;
  int wkc;
  auto next = std::chrono::steady_clock::now();
  auto period = std::chrono::milliseconds(1);

  std::vector<int32_t> motor_commands(ctx.slavecount, 0);
  std::vector<int32_t> motor_feedback(ctx.slavecount, 0);

  // Setting ModeOfOperation to CyclicSyncPositionMode
  for (int i = 1; i <= ctx.slavecount; i++) {
      CiA402Motor m{(CiA402_Inputs *)ctx.slavelist[i].inputs,
                    (CiA402_Outputs *)ctx.slavelist[i].outputs};
      m.set_mode_of_operation(
            CiA402Motor::ModeOfOperation::CyclicSyncPositionMode);
  }

  // Transitioning CiA402 State Machine to OPERATION_ENABLED
  int flag = 1;
  RCLCPP_INFO(logger, "Going to operation_enabled");
  while (flag) {
    flag = 0;
    for (int i = 1; i <= ctx.slavecount; i++) {
      CiA402Motor m{(CiA402_Inputs *)ctx.slavelist[i].inputs,
                    (CiA402_Outputs *)ctx.slavelist[i].outputs};

      RCLCPP_DEBUG(logger, "State of Motor %d: %s", i, m.state_as_string().c_str());
      if (!m.get_state().has_value()) {
	flag = 1;
	RCLCPP_INFO(logger, "Motor %d has no state", i);
      } else if (m.get_state().value() != CiA402Motor::State::OPERATION_ENABLED) {
        m.to_operation_enabled();
        flag = 1;
      } else if (m.get_state().value() == CiA402Motor::State::FAULT) {
	RCLCPP_INFO(logger, "Motor %d in fault. Fault handling not implemented. Exiting...", i);
	std::exit(EXIT_FAILURE);
      }
    }
    ecx_send_processdata(&ctx);
    ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
  }

  RCLCPP_INFO(logger, "All motors in operation_enabled");
  RCLCPP_INFO(logger, "Entering Cyclic Loop");

  while (running_) {
    next += period;
    // perf_read() is wait-free - returns immediately if no new data
    if (cmd_apsa.perf_read(motor_commands)) {
      // New commands received! Apply them to EtherCAT nodes
      for (int i = 1; i <= config.slavecount; i++) {
        CiA402_Outputs *motor_outputs =
          (CiA402_Outputs *)ctx.slavelist[i].outputs;
	CiA402_Inputs *motor_inputs =
          (CiA402_Inputs *)ctx.slavelist[i].inputs;

	// guard statement
	if (abs(motor_commands[i-1] - motor_inputs->PositionValue) > 100) {continue;}
	//write value
	motor_outputs->TargetPosition = motor_commands[i-1];
	//RCLCPP_INFO(logger, "TargetPosition: %d, PositionValue: %d", motor_outputs->TargetPosition, motor_inputs->PositionValue);
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
    }
    // If no new commands, EtherCAT nodes keep executing previous commands

    ecx_send_processdata(&ctx);
    wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);

    if (wkc != expectedWKC) {
      RCLCPP_WARN(logger, "Not all nodes responded");
    }

    for (int i = 1; i <= ctx.slavecount; i++) {
      CiA402_Inputs *motor_inputs =
          (CiA402_Inputs *)ctx.slavelist[i].inputs;
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

  // Shut down motors
  for (int i = 1; i <= ctx.slavecount; i++) {
    CiA402Motor m{(CiA402_Inputs *)ctx.slavelist[i].inputs,
                    (CiA402_Outputs *)ctx.slavelist[i].outputs};
    m.to_switch_on_disabled();
  }
  ecx_send_processdata(&ctx);
  wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);

  if (wkc != expectedWKC) {
    RCLCPP_WARN(logger, "Not all nodes responded");
  }
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

void ECManager::transition_ec(uint16 state) {
  std::string state_string = "Unknown";
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

  RCLCPP_INFO(logger, "Transition Ethercat State to %s", state_string.c_str());

  ctx.slavelist[0].state = state;
  ecx_writestate(&ctx, 0);
  ecx_statecheck(&ctx, 0, state, EC_TIMEOUTSTATE * 4);
  ecx_send_processdata(&ctx);
  ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
  if (ctx.slavelist[0].state != state) {
    RCLCPP_INFO(logger, "Not all nodes reached %d state", state);
    ecx_readstate(&ctx);
    for (int i = 1; i <= ctx.slavecount; i++)
      {
	if (ctx.slavelist[i].state != state)
	  {
	    RCLCPP_INFO(logger, "Node %d State=%2x StatusCode=%4x : %s",
			i, ctx.slavelist[i].state, ctx.slavelist[i].ALstatuscode, ec_ALstatuscode2string(ctx.slavelist[i].ALstatuscode));
	  }
      }
    std::exit(EXIT_FAILURE);
  }
}
