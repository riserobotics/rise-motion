#pragma once
#include <cstdint>
#include <mutex>
#include <rclcpp/logger.hpp>
#include <soem/soem.h>

#define IOMAP_SIZE 4096
class ECManager {
public:
  ECManager();
  ECManager(const std::string interface);
  void init_ec();
  void cyclic_loop();
  void get_motor_values(std::vector<uint32_t>& motor_values);
  void set_motor_values(std::vector<uint32_t>& motor_values);
private:
  void transition_to_operational();
  int expectedWKC;
  ecx_contextt ctx;
  uint8_t IOMap[IOMAP_SIZE];
  std::mutex ctx_mutex;
  const std::string interface;
  static rclcpp::Logger logger;
};
