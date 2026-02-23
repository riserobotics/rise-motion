#pragma once
#include <atomic>
#include <cstdint>
#include <rclcpp/logger.hpp>
#include <soem/soem.h>
#include <vector>

#include "apsa.hpp"

#define IOMAP_SIZE 4096

class ECManager {
public:
  ECManager();
  ECManager(const std::string interface);
  void init_ec();
  void cyclic_loop();
  void stop();

  // APSA-based motor value transfer (lock-free)
  bool get_motor_values_apsa(std::vector<int32_t>& motor_values);
  bool set_motor_values_apsa(const std::vector<int32_t>& motor_values);

  // Legacy mutex-based methods (deprecated)
  void get_motor_values(std::vector<int32_t>& motor_values);
  void set_motor_values(std::vector<int32_t>& motor_values);

private:
  uint16 transition_ec(uint16 state);
  void transition_to_operational();

  // EtherCAT context and configuration
  int expectedWKC;
  ecx_contextt ctx;
  uint8_t IOMap[IOMAP_SIZE];
  std::atomic<bool> running_{false};
  const std::string interface;
  static rclcpp::Logger logger;

  // APSA instances for lock-free communication
  // cmd_apsa: ROS → EtherCAT (motor commands)
  APSA<std::vector<int32_t>> cmd_apsa;

  // feedback_apsa: EtherCAT → ROS (motor feedback)
  APSA<std::vector<int32_t>> feedback_apsa;
};
