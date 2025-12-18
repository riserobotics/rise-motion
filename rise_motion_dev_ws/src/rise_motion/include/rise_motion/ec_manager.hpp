#pragma once
#include <cstdint>
#include <rclcpp/logger.hpp>
#include <rise_motion/ec_structs.hpp>
#include <rise_motion/snapshot.hpp>
#include <soem/soem.h>

#define IOMAP_SIZE 4096
class ECManager {
public:
  ECManager();
  ECManager(const std::string interface, Snapshot<MotorInputs> &inputs,
            Snapshot<MotorOutputs> &outputs);
  void init_ec();
  void cyclic_loop();

private:
  void transition_to_operational();
  void publish_outputs();
  void consume_inputs();
  int expectedWKC;
  ecx_contextt ctx;
  uint8_t IOMap[IOMAP_SIZE];

  const std::string interface;
  Snapshot<MotorInputs> &inputs;
  Snapshot<MotorOutputs> &outputs;
  static rclcpp::Logger logger;
};
