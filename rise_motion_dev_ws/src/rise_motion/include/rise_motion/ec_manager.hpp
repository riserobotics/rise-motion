#pragma once
#include <atomic>
#include <cstdint>
#include <rclcpp/logger.hpp>
#include <soem/soem.h>
#include <vector>
#include <chrono>

#include <rise_motion/cia402.hpp>
#include <rise_motion/apsa.hpp>

#define IOMAP_SIZE 4096

class ECManager {
public:
  ECManager();
  ECManager(const std::string interface, int cycle_period_ms);

  int init_ec();
  void cyclic_loop();
  bool is_running();
  void stop();

  // APSA-based motor value transfer (lock-free)
  bool get_motor_values_apsa(std::vector<int32_t>& motor_values);
  bool set_motor_values_apsa(const std::vector<int32_t>& motor_values);

  // Wrappers for SOEM ecx_SDOwrite, ecx_SDOread
  bool sdo_read(uint16 device_id, uint16 index, uint8 subindex, std::vector<uint8>& value);
  bool sdo_write(uint16 device_id, uint16 index, uint8 subindex, std::vector<uint8>& value);

private:
  uint16 transition_ec(uint16 state);
  bool transition_motors_to(CiA402Motor::State state);

  // EtherCAT context and configuration
  int expectedWKC;
  ecx_contextt ctx;
  uint8_t IOMap[IOMAP_SIZE];
  std::atomic<bool> running_{false};
  const std::string interface;
  const rclcpp::Logger logger;

  std::chrono::time_point<std::chrono::steady_clock> next;
  const std::chrono::duration<long, std::ratio<1,1000>> period; // period in ms

  // APSA instances for lock-free communication
  // cmd_apsa: ROS → EtherCAT (motor commands)
  APSA<std::vector<int32_t>> cmd_apsa;

  // feedback_apsa: EtherCAT → ROS (motor feedback)
  APSA<std::vector<int32_t>> feedback_apsa;
};
