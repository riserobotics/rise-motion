#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <limits>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rise_motion_messages/msg/admittance_debug.hpp>
#include <rise_motion_messages/msg/motor_feedback_full.hpp>
#include <rise_motion_messages/msg/motor_velocity.hpp>
#include <rise_motion_messages/srv/enable_ethercat_srv.hpp>

// AdmittanzNode: Implements admittance control law
//
// Input:  τ_int (interaction torque via load cell, from analog_input1)
// Output: velocity commands on /motor_commands_vel (MotorVelocity)
//
// Control law (transparent mode, K_v = 0):
//   M_v * v̇ + D_v * v = τ_int
//   Euler, dt = 1ms:
//     v̇  = (τ_int - D_v * v) / M_v
//     v  += v̇ * dt
//
// Force sensor calibration (ADC 0..65535, 0V..5V, 2.5V = 0N):
//   V = (adc / 65535.0 * 5.0) - 2.5   [V]
//   F = V * sensitivity_inv             [N]
//   F = clamp(F, -f_max, f_max)         [N] - sensor range limit
//   τ = F * lever_arm                   [Nm]
//
// NOTE: Currently publishes to /motor_commands_vel (not wired to motor).
// Wiring to actual motor commands requires enabling CyclicSyncVelocityMode
// on the drive first (TODO Woche 4).

class AdmittanzNode : public rclcpp::Node {
public:
  AdmittanzNode() : Node("admittanz_node") {
    // Admittance parameters
    declare_parameter("M_v", 1.0);
    declare_parameter("D_v", 10.0);
    declare_parameter("K_v", 0.0);

    // Force sensor calibration: ADC → Voltage → Force → Torque
    // ADC range: 0..65535 → 0V..5V, midpoint 32767.5 = 2.5V = 0N
    declare_parameter("sensitivity_inv", 100.0);   // N/V (placeholder, calibrate Woche 3)
    declare_parameter("lever_arm", 0.1);           // m   (placeholder)

    // Safety limits
    declare_parameter("f_max", 500.0);     // N    - input clipping (sensor range)
    declare_parameter("vel_max", 1000.0);  // inc/s - output clipping
    declare_parameter("dvel_max", 500.0);  // inc/s per step - jerk limit

    // Position limits (enc-inc): motor stops if position exceeds these bounds
    // Defaults: no effect (full int32 range)
    declare_parameter("pos_min",
                      static_cast<int64_t>(std::numeric_limits<int32_t>::min()));
    declare_parameter("pos_max",
                      static_cast<int64_t>(std::numeric_limits<int32_t>::max()));

    feedback_sub_ =
        create_subscription<rise_motion_messages::msg::MotorFeedbackFull>(
            "motor_feedback_full", 10,
            [this](rise_motion_messages::msg::MotorFeedbackFull::SharedPtr msg) {
              feedbackCallback(msg);
            });

    cmd_pub_ = create_publisher<rise_motion_messages::msg::MotorVelocity>(
        "motor_commands_vel", 10);

    debug_pub_ = create_publisher<rise_motion_messages::msg::AdmittanceDebug>(
        "admittance_debug", 10);

    compute_timer_ = create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&AdmittanzNode::computeAdmittance, this));

    client_ = create_client<rise_motion_messages::srv::EnableEthercatSrv>(
        "enable_ethercat");

    RCLCPP_INFO(get_logger(), "AdmittanzNode initialized (publishing to /motor_commands_vel)");
  }

  ~AdmittanzNode() { RCLCPP_INFO(get_logger(), "AdmittanzNode shutting down"); }

  int request_enable_ethercat() {
    RCLCPP_INFO(get_logger(), "Requesting Enable Ethercat");
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(),
                     "client interrupted while waiting for service to appear.");
        return 1;
      }
      RCLCPP_INFO(get_logger(), "waiting for service to appear...");
    }
    auto request = std::make_shared<
        rise_motion_messages::srv::EnableEthercatSrv::Request>();
    request->enable = true;
    auto result_future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "service call failed");
      client_->remove_pending_request(result_future);
      return 1;
    }
    return result_future.get()->status_enable;
  }

private:
  rclcpp::Subscription<rise_motion_messages::msg::MotorFeedbackFull>::SharedPtr
      feedback_sub_;
  rclcpp::Publisher<rise_motion_messages::msg::MotorVelocity>::SharedPtr
      cmd_pub_;
  rclcpp::Publisher<rise_motion_messages::msg::AdmittanceDebug>::SharedPtr
      debug_pub_;
  rclcpp::Client<rise_motion_messages::srv::EnableEthercatSrv>::SharedPtr
      client_;
  rclcpp::TimerBase::SharedPtr compute_timer_;

  // State per motor
  std::vector<int32_t>  pos_current_;
  std::vector<double>   velocity_;
  std::vector<double>   displacement_;
  std::vector<uint16_t> analog_input1_;
  bool has_feedback_{false};

  static constexpr double dt_ = 0.001;  // 1ms

  void feedbackCallback(
      rise_motion_messages::msg::MotorFeedbackFull::SharedPtr msg) {
    if (!has_feedback_) {
      size_t n = msg->positions.size();
      pos_current_.resize(n, 0);
      velocity_.resize(n, 0.0);
      displacement_.resize(n, 0.0);
      analog_input1_.resize(n, 32768);  // init to midpoint (0N)
      has_feedback_ = true;
      RCLCPP_INFO(get_logger(), "Got first feedback (%zu motors)", n);
    }
    pos_current_  = msg->positions;
    analog_input1_ = msg->analog_input1;
  }

  void computeAdmittance() {
    if (!has_feedback_) return;

    // Read parameters every cycle (allows live tuning via ros2 param set)
    const double M_v             = get_parameter("M_v").as_double();
    const double D_v             = get_parameter("D_v").as_double();
    const double K_v             = get_parameter("K_v").as_double();
    const double sensitivity_inv = get_parameter("sensitivity_inv").as_double();
    const double lever_arm       = get_parameter("lever_arm").as_double();
    const double f_max           = get_parameter("f_max").as_double();
    const double vel_max         = get_parameter("vel_max").as_double();
    const double dvel_max        = get_parameter("dvel_max").as_double();
    const auto   pos_min         = static_cast<int32_t>(get_parameter("pos_min").as_int());
    const auto   pos_max         = static_cast<int32_t>(get_parameter("pos_max").as_int());

    const size_t n = pos_current_.size();
    auto msg = rise_motion_messages::msg::MotorVelocity();
    msg.velocities.resize(n);

    auto dbg = rise_motion_messages::msg::AdmittanceDebug();
    dbg.adc_voltage.resize(n);
    dbg.force.resize(n);
    dbg.force_clipped.resize(n);
    dbg.torque_raw.resize(n);
    dbg.vdot.resize(n);
    dbg.velocity_raw.resize(n);
    dbg.velocity_output.resize(n);
    dbg.displacement.resize(n);

    for (size_t i = 0; i < n; ++i) {
      // ADC → Voltage → Force → Torque
      // ADC: 0..65535 → 0V..5V, 2.5V = 0N (bipolar sensor)
      double V     = (static_cast<double>(analog_input1_[i]) / 65535.0 * 5.0) - 2.5;
      double F_raw = V * sensitivity_inv;
      double F     = std::clamp(F_raw, -f_max, f_max);  // sensor range limit [N]
      double tau   = F * lever_arm;                       // [Nm]

      // Admittance Euler integration
      double vdot   = (tau - D_v * velocity_[i] - K_v * displacement_[i]) / M_v;
      double v_prev = velocity_[i];
      velocity_[i] += vdot * dt_;
      displacement_[i] += velocity_[i] * dt_;

      // Output: jerk limit then velocity clamp
      velocity_[i] = std::clamp(velocity_[i],
                                 v_prev - dvel_max * dt_,
                                 v_prev + dvel_max * dt_);
      velocity_[i] = std::clamp(velocity_[i], -vel_max, vel_max);

      // Position limits: stop motion toward a breached limit
      if ((pos_current_[i] <= pos_min && velocity_[i] < 0.0) ||
          (pos_current_[i] >= pos_max && velocity_[i] > 0.0)) {
        velocity_[i] = 0.0;
      }

      msg.velocities[i] = static_cast<int32_t>(velocity_[i]);

      dbg.adc_voltage[i]   = V;
      dbg.force[i]         = F_raw;
      dbg.force_clipped[i] = F;
      dbg.torque_raw[i]    = tau;
      dbg.vdot[i]          = vdot;
      dbg.velocity_raw[i]  = v_prev + vdot * dt_;
      dbg.velocity_output[i] = velocity_[i];
      dbg.displacement[i]  = displacement_[i];
    }

    cmd_pub_->publish(msg);
    debug_pub_->publish(dbg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AdmittanzNode>();
  while (!node->request_enable_ethercat()) {
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
