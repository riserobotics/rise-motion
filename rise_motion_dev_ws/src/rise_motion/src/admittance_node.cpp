#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <limits>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rise_motion_messages/msg/admittance_debug.hpp>
#include <rise_motion_messages/msg/motor_feedback_full.hpp>
#include <rise_motion_messages/msg/motor_velocity.hpp>
#include <rise_motion_messages/srv/enable_ethercat_srv.hpp>
#include <std_msgs/msg/u_int16.hpp>

// AdmittanzNode: Implements admittance control law
//
// Input:  τ_int (interaction torque via load cell, from analog_input1)
// Output: velocity commands on /motor_commands_vel (MotorVelocity, enc-inc/s)
//
// Control law (transparent mode, K_v = 0):
//   M_v * v̇ + D_v * v = τ_int
//   Euler, dt = 1ms:
//     v̇  = (τ_int - D_v * v) / M_v     [rad/s^2]
//     v  += v̇ * dt                      [rad/s]
//     q  += v  * dt                      [rad]
//
// All internal state is in physical units (rad, rad/s, rad/s^2).
// Conversion to enc-inc/s happens only on publish via enc_per_rad.
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
    // Admittance parameters (SI units)
    // M_v: virtual inertia  – higher = slower response, more "mass-like" feel
    // D_v: virtual damping  – higher = smaller velocity at same torque, more resistance
    //                         steady-state: v_eq = tau / D_v
    // K_v: virtual stiffness – 0 for transparent mode, >0 adds restoring force to neutral
    declare_parameter("M_v", 3.0);    // [kg*m^2]   realistic knee joint inertia: 0.3..0.8
    declare_parameter("D_v", 10.0);    // [Nm*s/rad] v_eq = 25Nm / 5.0 = 5 rad/s (286 deg/s)
    declare_parameter("K_v", 0.0);    // [Nm/rad]   0 = transparent mode

    // Force sensor calibration: ADC → Voltage → Force → Torque
    // ADC range: 0..65535 → 0V..5V, midpoint 2.5V = 0N
    declare_parameter("sensitivity_inv", 10.0);  // [N/V]  placeholder, calibrate Woche 3
    declare_parameter("lever_arm", 0.1);          // [m]    placeholder

    // Safety limits (SI units)
    declare_parameter("f_max", 200.0);   // [N]       typical cuff load cell range
    declare_parameter("vel_max", 3.0);   // [rad/s]   ~172 deg/s, reasonable for transparent mode
    declare_parameter("dvel_max", 10.0); // [rad/s^2] gentle jerk limit, avoids abrupt steps

    // Encoder conversion: enc-inc per radian (motor-specific, depends on resolution + gear ratio)
    // enc_per_rad = ENCODER_RESOLUTION * GEAR_RATIO / (2*pi)
    // Default: Hüfte AA (2560 * 160 / 2pi ≈ 65306)
    declare_parameter("enc_per_rad", 65306.0);

    // Position limits (enc-inc, kept in motor units since pos_current_ comes from PDO)
    // Defaults: no effect (full int32 range)
    declare_parameter("pos_min",
                      static_cast<int64_t>(std::numeric_limits<int32_t>::min()));
    declare_parameter("pos_max",
                      static_cast<int64_t>(std::numeric_limits<int32_t>::max()));

    // Control loop timing
    // compute_period_ms defines the timer rate (independent of feedback publish rate).
    // dt clamps guard against jitter (dtmin) and missed feedback packets (dtmax).
    // dtmax=20ms covers one missed 10ms feedback packet.
    declare_parameter("compute_period_ms", 1);   // [ms]  timer period; 1ms = 1kHz
    declare_parameter("dt_min_ms", 0.5);         // [ms]  ignore spuriously short steps
    declare_parameter("dt_max_ms", 20.0);        // [ms]  clamp missed-packet steps

    // Fake sensor override: if true, /analog_input_override topic (std_msgs/UInt16, ADC ticks 0..65535)
    // replaces analog_input1 for all motors. Same format as the real sensor — full calibration pipeline active.
    // 0 = 0V = -2.5V after offset = max negative force
    // 32768 = 2.5V = 0N (neutral)
    // 65535 = 5V = max positive force
    declare_parameter("use_force_override", false);

    analog_override_sub_ = create_subscription<std_msgs::msg::UInt16>(
        "analog_input_override", 10,
        [this](std_msgs::msg::UInt16::SharedPtr msg) {
          analog_override_value_ = msg->data;
        });

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

    const int period_ms = get_parameter("compute_period_ms").as_int();
    compute_timer_ = create_wall_timer(
        std::chrono::milliseconds(period_ms),
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
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr analog_override_sub_;
  std::atomic<uint16_t> analog_override_value_{32768};  // default: 2.5V = 0N
  rclcpp::Publisher<rise_motion_messages::msg::MotorVelocity>::SharedPtr
      cmd_pub_;
  rclcpp::Publisher<rise_motion_messages::msg::AdmittanceDebug>::SharedPtr
      debug_pub_;
  rclcpp::Client<rise_motion_messages::srv::EnableEthercatSrv>::SharedPtr
      client_;
  rclcpp::TimerBase::SharedPtr compute_timer_;

  // State per motor (all in physical units: rad/s, rad)
  std::vector<int32_t>  pos_current_;
  std::vector<double>   velocity_;     // [rad/s]
  std::vector<double>   displacement_; // [rad]
  std::vector<uint16_t> analog_input1_;
  bool has_feedback_{false};
  bool new_feedback_available_{false};  // true after each feedbackCallback, cleared by computeAdmittance

  // Timing: measure actual dt each cycle instead of assuming fixed period
  std::chrono::steady_clock::time_point last_compute_time_;
  bool first_compute_{true};

  void feedbackCallback(
      rise_motion_messages::msg::MotorFeedbackFull::SharedPtr msg) {
    const size_t n = msg->positions.size();
    if (!has_feedback_) {
      pos_current_.resize(n, 0);
      velocity_.resize(n, 0.0);
      displacement_.resize(n, 0.0);
      analog_input1_.resize(n, 32768);  // init to midpoint (0N)
      has_feedback_ = true;
      RCLCPP_INFO(get_logger(), "Got first feedback (%zu motors)", n);
    } else if (n != pos_current_.size()) {
      // Motor count changed (e.g. EtherCAT reconnect) — reinitialize state
      RCLCPP_WARN(get_logger(), "Feedback size changed %zu → %zu, reinitializing state",
                  pos_current_.size(), n);
      pos_current_.assign(n, 0);
      velocity_.assign(n, 0.0);
      displacement_.assign(n, 0.0);
      analog_input1_.assign(n, 32768);
    }
    pos_current_            = msg->positions;
    analog_input1_          = msg->analog_input1;
    new_feedback_available_ = true;
  }

  void computeAdmittance() {
    if (!has_feedback_ || !new_feedback_available_) return;
    new_feedback_available_ = false;

    // Measure actual dt since last compute
    auto now = std::chrono::steady_clock::now();
    if (first_compute_) {
      last_compute_time_ = now;
      first_compute_ = false;
      return;
    }
    double dt = std::chrono::duration<double>(now - last_compute_time_).count();
    last_compute_time_ = now;
    const double dtmin = get_parameter("dt_min_ms").as_double() * 1e-3;
    const double dtmax = get_parameter("dt_max_ms").as_double() * 1e-3;
    dt = std::clamp(dt, dtmin, dtmax);

    // Read parameters every cycle (allows live tuning via ros2 param set)
    const double M_v             = get_parameter("M_v").as_double();
    if (M_v <= 0.0) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Invalid M_v=%.4f (must be > 0), skipping cycle", M_v);
      return;
    }
    const double D_v             = get_parameter("D_v").as_double();
    const double K_v             = get_parameter("K_v").as_double();
    const double sensitivity_inv = get_parameter("sensitivity_inv").as_double();
    const double lever_arm       = get_parameter("lever_arm").as_double();
    const double f_max           = get_parameter("f_max").as_double();
    const double vel_max         = get_parameter("vel_max").as_double();
    const double dvel_max        = get_parameter("dvel_max").as_double();
    const double enc_per_rad     = get_parameter("enc_per_rad").as_double();
    const auto   pos_min         = static_cast<int32_t>(get_parameter("pos_min").as_int());
    const auto   pos_max         = static_cast<int32_t>(get_parameter("pos_max").as_int());

    const size_t n = pos_current_.size();
    auto msg = rise_motion_messages::msg::MotorVelocity();
    msg.header.stamp = get_clock()->now();
    msg.velocities.resize(n);

    constexpr double rad_to_deg = 180.0 / M_PI;

    auto dbg = rise_motion_messages::msg::AdmittanceDebug();
    dbg.header.stamp = get_clock()->now();
    dbg.adc_voltage.resize(n);
    dbg.force.resize(n);
    dbg.force_clipped.resize(n);
    dbg.torque_raw.resize(n);
    dbg.vdot.resize(n);
    dbg.velocity_raw.resize(n);
    dbg.velocity_output.resize(n);
    dbg.displacement.resize(n);
    dbg.vdot_deg.resize(n);
    dbg.velocity_raw_deg.resize(n);
    dbg.velocity_output_deg.resize(n);
    dbg.displacement_deg.resize(n);

    for (size_t i = 0; i < n; ++i) {
      // ADC → Voltage → Force → Torque
      // ADC: 0..65535 → 0V..5V, midpoint 2.5V = 0N (bipolar sensor)
      // Subtract 2.5V so that V=0 means no force. Without this, the controller
      // would see F=250N at rest and drive the motor even with no interaction.
      const uint16_t adc = get_parameter("use_force_override").as_bool()
                           ? analog_override_value_.load()
                           : analog_input1_[i];
      double V     = (static_cast<double>(adc) / 65535.0 * 5.0) - 2.5;
      double F_raw = V * sensitivity_inv;
      double F   = std::clamp(F_raw, -f_max, f_max);  // sensor range limit [N]
      double tau = F * lever_arm;                       // [Nm]

      // Admittance Euler integration (all in rad/s, rad)
      double vdot   = (tau - D_v * velocity_[i] - K_v * displacement_[i]) / M_v;
      double v_prev = velocity_[i];
      velocity_[i] += vdot * dt;
      displacement_[i] += velocity_[i] * dt;

      // Output safety: jerk limit then velocity clamp
      velocity_[i] = std::clamp(velocity_[i],
                                 v_prev - dvel_max * dt,
                                 v_prev + dvel_max * dt);
      velocity_[i] = std::clamp(velocity_[i], -vel_max, vel_max);

      // Position limits (enc-inc): stop motion toward a breached limit
      if ((pos_current_[i] <= pos_min && velocity_[i] < 0.0) ||
          (pos_current_[i] >= pos_max && velocity_[i] > 0.0)) {
        velocity_[i] = 0.0;
      }

      // Convert rad/s → enc-inc/s for motor command
      msg.velocities[i] = static_cast<int32_t>(velocity_[i] * enc_per_rad);

      dbg.adc_voltage[i]         = V;
      dbg.force[i]               = F_raw;
      dbg.force_clipped[i]       = F;
      dbg.torque_raw[i]          = tau;
      dbg.vdot[i]                = vdot;
      dbg.velocity_raw[i]        = v_prev + vdot * dt;
      dbg.velocity_output[i]     = velocity_[i];
      dbg.displacement[i]        = displacement_[i];
      dbg.vdot_deg[i]            = vdot              * rad_to_deg;
      dbg.velocity_raw_deg[i]    = (v_prev + vdot * dt) * rad_to_deg;
      dbg.velocity_output_deg[i] = velocity_[i]      * rad_to_deg;
      dbg.displacement_deg[i]    = displacement_[i]  * rad_to_deg;
    }

    cmd_pub_->publish(msg);
    debug_pub_->publish(dbg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AdmittanzNode>();
  while (rclcpp::ok() && !node->request_enable_ethercat()) {
  }
  if (!rclcpp::ok()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
