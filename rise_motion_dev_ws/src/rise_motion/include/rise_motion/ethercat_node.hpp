#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rise_motion/ec_manager.hpp>
#include <rise_motion_messages/msg/motor_positions.hpp>
#include <rise_motion_messages/srv/enable_ethercat_srv.hpp>
#include <thread>

class EthercatNode : public rclcpp::Node {
public:
  explicit EthercatNode(ECManager& ec_manager);
  ~EthercatNode();

private:
  ECManager& ec_manager_;
  std::unique_ptr<std::thread> ec_thread_;
  bool ethercat_enabled_{false};

  rclcpp::Subscription<rise_motion_messages::msg::MotorPositions>::SharedPtr cmd_sub_;
  rclcpp::Publisher<rise_motion_messages::msg::MotorPositions>::SharedPtr feedback_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::Service<rise_motion_messages::srv::EnableEthercatSrv>::SharedPtr enable_srv_;

  void commandCallback(const rise_motion_messages::msg::MotorPositions::SharedPtr msg);
  void publishFeedback();
  void enableServiceCallback(
    const std::shared_ptr<rise_motion_messages::srv::EnableEthercatSrv::Request> request,
    std::shared_ptr<rise_motion_messages::srv::EnableEthercatSrv::Response> response);
};
