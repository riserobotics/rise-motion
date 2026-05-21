#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <vector>
#include <rclcpp/logger.hpp>

#include <rise_motion/iec_manager.hpp>
#include <rise_motion/apsa.hpp>

class MockECManager : public IECManager {
public:
  explicit MockECManager(int cycle_period_ms = 1, int num_motors = 1,
                         float alpha = 1.0f);

  int  init_ec() override;
  void cyclic_loop() override;
  bool is_running() override;
  void stop() override;

  bool get_motor_values_apsa(std::vector<int32_t>& motor_values) override;
  bool set_motor_values_apsa(const std::vector<int32_t>& motor_values) override;
  bool get_full_feedback_apsa(std::vector<MotorFeedbackData>& feedback) override;
  bool set_motor_velocity_apsa(const std::vector<int32_t>& velocities) override;
  bool set_torque_offset_apsa(const std::vector<int16_t>& offsets) override;
  void set_operation_mode(int8_t mode) override;

  bool sdo_read(uint16_t device_id, uint16_t index,
                uint8_t subindex, std::vector<uint8_t>& value) override;
  bool sdo_write(uint16_t device_id, uint16_t index,
                 uint8_t subindex, std::vector<uint8_t>& value) override;

private:
  const int num_motors_;
  const float alpha_;
  std::atomic<bool> running_{false};
  const rclcpp::Logger logger_;

  std::vector<int32_t> mock_positions_;
  std::vector<double> mock_positions_float_;
  std::vector<int32_t> motor_commands_;
  std::vector<int32_t> velocity_commands_;
  uint64_t tick_count_{0};

  std::chrono::time_point<std::chrono::steady_clock> next_;
  const std::chrono::duration<long, std::ratio<1, 1000>> period_;

  APSA<std::vector<int32_t>> cmd_apsa_;
  APSA<std::vector<int32_t>> vel_cmd_apsa_;
  APSA<std::vector<int32_t>> feedback_apsa_;
  APSA<std::vector<MotorFeedbackData>> full_feedback_apsa_;
};
