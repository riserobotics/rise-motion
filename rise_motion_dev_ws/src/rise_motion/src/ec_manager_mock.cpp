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
      motor_commands_(num_motors, 0),
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

    // Pull latest motor commands from ROS thread (wait-free)
    cmd_apsa_.perf_read(motor_commands_);

    // Simulate motor: track commanded position with configurable lag
    for (int i = 0; i < num_motors_; ++i) {
      mock_positions_[i] += static_cast<int32_t>(
          alpha_ * static_cast<float>(motor_commands_[i] - mock_positions_[i]));
    }

    // Publish feedback to ROS thread (wait-free)
    feedback_apsa_.perf_write(mock_positions_);

    // Simulate full PDO feedback: analog_input1 = 1Hz sinus (±2.5V range)
    // ADC range: 0=Umin(-5V), 65535=Umax(+5V), midpoint=32768(0V)
    double t = tick_count_ * period_.count() * 1e-3;
    uint16_t analog_sim = static_cast<uint16_t>(
        32768.0 + 16384.0 * std::sin(2.0 * M_PI * 1.0 * t));

    std::vector<MotorFeedbackData> full_feedback(num_motors_);
    for (int i = 0; i < num_motors_; ++i) {
      full_feedback[i].position      = mock_positions_[i];
      full_feedback[i].analog_input1 = analog_sim;
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

bool MockECManager::sdo_read(uint16_t /*device_id*/, uint16_t /*index*/,
                              uint8_t /*subindex*/, std::vector<uint8_t>& value) {
  value = {0};
  return true;
}

bool MockECManager::sdo_write(uint16_t /*device_id*/, uint16_t /*index*/,
                               uint8_t /*subindex*/, std::vector<uint8_t>& /*value*/) {
  return true;
}
