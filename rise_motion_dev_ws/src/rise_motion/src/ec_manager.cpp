#include <chrono>
#include <cstdint>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rise_motion/ec_manager.hpp>
#include <rise_motion/ec_structs.hpp>
#include <rise_motion/snapshot.hpp>
#include <soem/soem.h>
#include <string>
#include <thread>
#include <vector>

// expected config, needs to be retrieved from config node
struct {
  int slavecount = 1;
  ec_slavet slavelist[1] = {{.name = "a name"}};
} config;

ECManager::ECManager(const std::string interface, Snapshot<MotorInputs> &inputs,
                     Snapshot<MotorOutputs> &outputs)
    : interface(interface), inputs(inputs), outputs(outputs) {}

void ECManager::init_ec() {
  int ret;
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
      RCLCPP_WARN(logger, "Node %d: Name does not match", i);
    }
  }
  // Now all nodes should be in safe op
}
void ECManager::transition_to_operational() {
  RCLCPP_INFO(logger, "Entering operational mode");
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
  int wkc;
  auto next = std::chrono::steady_clock::now();
  auto period = std::chrono::milliseconds(1);
  for (;;) {
    next += period;
    ecx_send_processdata(&ctx);
    wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);

    publish_outputs(); // share with ros node
    consume_inputs();  // get from ros node

    if (wkc != expectedWKC) {
      RCLCPP_WARN(logger, "Not all nodes responded");
    }
    std::this_thread::sleep_until(next);
  }
}

void ECManager::publish_outputs() {
  MotorOutputs new_outputs;
  memcpy(&new_outputs, ctx.slavelist[1].outputs, sizeof(MotorOutputs));
  outputs.write(new_outputs);
}

void ECManager::consume_inputs() {
  MotorInputs new_inputs;
  inputs.read(new_inputs);
  memcpy(ctx.slavelist[1].inputs, &new_inputs, sizeof(MotorInputs));
}

rclcpp::Logger ECManager::logger = rclcpp::get_logger("ECManager");
