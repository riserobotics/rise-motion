#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rise_motion/ec_manager.hpp>
#include <rise_motion/ec_structs.hpp>
#include <rise_motion/cia402.hpp>
#include <soem/soem.h>
#include <string>
#include <thread>
#include <vector>

// expected config, needs to be retrieved from config node
struct {
  int slavecount = 1;
  ec_slavet slavelist[1] = {{.name = "a name"}};
} config;

ECManager::ECManager(const std::string interface) : interface(interface) {}

void ECManager::init_ec() {
  int ret;
  RCLCPP_INFO(logger, "Connecting to %s", interface.c_str());
  ctx.packedMode = TRUE;
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

  RCLCPP_INFO(logger, "Mapping IO");
  ret = ecx_config_map_group(&ctx, IOMap, 0);
  if (ret > IOMAP_SIZE) {
    RCLCPP_WARN(logger, "Couldn't map IO: Buffer to small");
    std::exit(EXIT_FAILURE);
  }

  expectedWKC = ctx.grouplist[0].outputsWKC * 2 + ctx.grouplist[0].inputsWKC;
  RCLCPP_INFO(logger, "Configuring ditributed clock");
  ecx_configdc(&ctx);

  ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

  // Check if nodes have valid outputs
  ecx_send_processdata(&ctx);
  ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
  // TODO: Check if nodes have valid outputs
  for (int i = 1; i <= ctx.slavecount; i++) {
    if (strcmp(config.slavelist[i].name, ctx.slavelist[i].name)) {
      RCLCPP_WARN(logger, "Node %d: Name does not match: %s != %s", i, config.slavelist[i].name, ctx.slavelist[i].name);
    }
  }
  // Now all nodes should be in safe op
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

  int flag = 1;
  std::cout << "Going to operation_enabled\n";
  while (flag) {
  flag = 0;
    for (int i = 1; i < config.slavecount; i++) {
        CiA402Motor m{(CiA402_Inputs*)ctx.slavelist[i].inputs, (CiA402_Outputs*)ctx.slavelist[i].outputs};
        if (m.get_state().value() != CiA402Motor::State::OPERATION_ENABLED) {
          m.to_operation_enabled();
          flag = 1;
        } else {
          m.set_mode_of_operation(CiA402Motor::ModeOfOperation::ProfilePositionMode);
        }
      }
  }
    std::cout << "All motor in operation_enabled\n";

  while (running_) {
    next += period;
    // perf_read() is wait-free - returns immediately if no new data
    if (cmd_apsa.perf_read(motor_commands)) {
      // New commands received! Apply them to EtherCAT slaves
      RCLCPP_INFO(logger, "New Motor Positions");
      for (int i = 0; i < config.slavecount; i++) {
        RCLCPP_INFO(logger, "Writing %d to Motor %d", motor_commands[i], i+1);
        inputs *motor_inputs = (inputs *)ctx.slavelist[i + 1].inputs;
        motor_inputs->TargetPosition = motor_commands[i];
      }
    }
    // If no new commands, EtherCAT slaves keep executing previous commands

    ecx_send_processdata(&ctx);
    wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);

    if (wkc != expectedWKC) {
      RCLCPP_WARN(logger, "Not all nodes responded");
    }

    for (int i = 0; i < config.slavecount; i++) {
      outputs *motor_outputs = (outputs *)ctx.slavelist[i + 1].outputs;
      motor_feedback[i] = motor_outputs->PositionValue;
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


bool ECManager::set_motor_values_apsa(const std::vector<int32_t> &motor_values) {
  // comm_write() queues the data for the EtherCAT loop to pick up
  return cmd_apsa.comm_write(motor_values);
}

rclcpp::Logger ECManager::logger = rclcpp::get_logger("ECManager");
