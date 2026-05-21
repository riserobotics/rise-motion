#include <cmath>
#include <cstdlib>
#include <thread>
#include <rclcpp/logging.hpp>

#include <rise_motion/ec_manager_mock.hpp>

MockECManager::MockECManager(int cycle_period_ms, int num_motors, float alpha)
    : num_motors_(num_motors),
      alpha_(alpha),
      logger_(rclcpp::get_logger("MockECManager")),
      mock_positions_(num_motors, 0),
      mock_positions_float_(num_motors, 0.0),
      motor_commands_(num_motors, 0),
      velocity_commands_(num_motors, 0),
      period_(cycle_period_ms) {}

int MockECManager::init_ec() {
  RCLCPP_INFO(logger_, "Mock ECManager: %d motor(s) ready", num_motors_);
  return EXIT_SUCCESS;
}

void MockECManager::cyclic_loop() {
  running_ = true;
  next_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(logger_, "Mock ECManager: entering cyclic loop at %d ms period",
              (int)period_.count());

  while (running_) {
    next_ += period_;

    // Pull latest commands from ROS thread (wait-free)
    cmd_apsa_.perf_read(motor_commands_);
    vel_cmd_apsa_.perf_read(velocity_commands_);

    // Integrate velocity commands into position with floating-point precision
    const double dt = period_.count() * 1e-3;
    for (int i = 0; i < num_motors_; ++i) {
      mock_positions_float_[i] += static_cast<double>(velocity_commands_[i]) * dt;
      mock_positions_[i] = static_cast<int32_t>(std::round(mock_positions_float_[i]));
    }

    // Publish feedback to ROS thread (wait-free)
    feedback_apsa_.perf_write(mock_positions_);

    std::vector<MotorFeedbackData> full_feedback(num_motors_);
    for (int i = 0; i < num_motors_; ++i) {
      full_feedback[i].position = mock_positions_[i];
    }
    full_feedback_apsa_.perf_write(full_feedback);

    tick_count_++;
    std::this_thread::sleep_until(next_);
  }
}

bool MockECManager::is_running() {
  return running_;
}

void MockECManager::stop() {
  running_ = false;
}

bool MockECManager::get_motor_values_apsa(std::vector<int32_t>& motor_values) {
  return feedback_apsa_.comm_read(motor_values);
}

bool MockECManager::set_motor_values_apsa(const std::vector<int32_t>& motor_values) {
  return cmd_apsa_.comm_write(motor_values);
}

bool MockECManager::get_full_feedback_apsa(std::vector<MotorFeedbackData>& feedback) {
  return full_feedback_apsa_.comm_read(feedback);
}

bool MockECManager::set_motor_velocity_apsa(const std::vector<int32_t>& velocities) {
  return vel_cmd_apsa_.comm_write(velocities);
}

bool MockECManager::set_torque_offset_apsa(const std::vector<int16_t>& /*offsets*/) {
  return true;
}

void MockECManager::set_operation_mode(int8_t mode) {
  RCLCPP_INFO(logger_, "Mock: operation mode set to %d", mode);
}

bool MockECManager::sdo_read(uint16_t /*device_id*/, uint16_t /*index*/,
                              uint8_t /*subindex*/, std::vector<uint8_t>& value) {
  value = {0};
  return true;
}

bool MockECManager::sdo_write(uint16_t /*device_id*/, uint16_t /*index*/,
                               uint8_t /*subindex*/, std::vector<uint8_t>& /*value*/) {
  return true;
}
