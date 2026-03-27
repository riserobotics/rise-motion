#pragma once
#include <atomic>
#include <cstdint>
#include <rclcpp/logger.hpp>
#include <soem/soem.h>
#include <vector>
#include <chrono>

#include <rise_motion/iec_manager.hpp>
#include <rise_motion/cia402.hpp>
#include <rise_motion/apsa.hpp>

#define IOMAP_SIZE 4096

class ECManager : public IECManager {
public:
  ECManager();
  ECManager(const std::string interface, int cycle_period_ms);

  int init_ec() override;
  void cyclic_loop() override;
  bool is_running() override;
  void stop() override;

  // APSA-based motor value transfer (lock-free)
  bool get_motor_values_apsa(std::vector<int32_t>& motor_values) override;
  bool set_motor_values_apsa(const std::vector<int32_t>& motor_values) override;
  bool get_full_feedback_apsa(std::vector<MotorFeedbackData>& feedback) override;

  // Wrappers for SOEM ecx_SDOwrite, ecx_SDOread
  bool sdo_read(uint16_t device_id, uint16_t index, uint8_t subindex, std::vector<uint8_t>& value) override;
  bool sdo_write(uint16_t device_id, uint16_t index, uint8_t subindex, std::vector<uint8_t>& value) override;

private:
  uint16 transition_ec(uint16 state);
  bool transition_motors_to(CiA402Motor::State state);
  void shutdown();

  // EtherCAT context and configuration
  int expectedWKC;
  ecx_contextt ctx;
  uint8_t IOMap[IOMAP_SIZE];
  std::vector<CiA402Motor> motors;

  std::atomic<bool> running_{false};
  const std::string interface;
  const rclcpp::Logger logger;

  std::chrono::time_point<std::chrono::steady_clock> next;
  const std::chrono::duration<long, std::ratio<1,1000>> period; // period in ms

  // APSA instances for lock-free communication
  // cmd_apsa: ROS → EtherCAT (motor commands)
  APSA<std::vector<int32_t>> cmd_apsa;

  // feedback_apsa: EtherCAT → ROS (motor positions only, für /motor_feedback)
  APSA<std::vector<int32_t>> feedback_apsa;

  // full_feedback_apsa: EtherCAT → ROS (alle PDO-Felder, für /motor_feedback_full)
  APSA<std::vector<MotorFeedbackData>> full_feedback_apsa;
};
