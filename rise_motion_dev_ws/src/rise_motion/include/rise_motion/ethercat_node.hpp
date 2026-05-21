#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rise_motion/iec_manager.hpp>
#include <rise_motion_messages/msg/motor_positions.hpp>
#include <rise_motion_messages/msg/motor_feedback_full.hpp>
#include <rise_motion_messages/msg/motor_velocity.hpp>
#include <rise_motion_messages/msg/motor_torque_offset.hpp>
#include <rise_motion_messages/srv/enable_ethercat_srv.hpp>
#include <rise_motion_messages/srv/sdo_read_srv.hpp>
#include <rise_motion_messages/srv/sdo_write_srv.hpp>
#include <rise_motion_messages/srv/set_operation_mode_srv.hpp>
#include <thread>

class EthercatNode : public rclcpp::Node {
public:
  explicit EthercatNode(IECManager& ec_manager);
  ~EthercatNode();

private:
  IECManager& ec_manager_;
  std::unique_ptr<std::thread> ec_thread_;
  bool ethercat_enabled_{false};

  rclcpp::Subscription<rise_motion_messages::msg::MotorPositions>::SharedPtr cmd_sub_;
  rclcpp::Subscription<rise_motion_messages::msg::MotorVelocity>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<rise_motion_messages::msg::MotorTorqueOffset>::SharedPtr torque_offset_sub_;
  rclcpp::Publisher<rise_motion_messages::msg::MotorPositions>::SharedPtr feedback_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::Publisher<rise_motion_messages::msg::MotorFeedbackFull>::SharedPtr full_feedback_pub_;
  rclcpp::TimerBase::SharedPtr full_feedback_timer_;
  rclcpp::Service<rise_motion_messages::srv::EnableEthercatSrv>::SharedPtr enable_srv_;
  rclcpp::Service<rise_motion_messages::srv::SDOReadSrv>::SharedPtr sdo_read_srv_;
  rclcpp::Service<rise_motion_messages::srv::SDOWriteSrv>::SharedPtr sdo_write_srv_;
  rclcpp::Service<rise_motion_messages::srv::SetOperationModeSrv>::SharedPtr mode_srv_;


  void commandCallback(const rise_motion_messages::msg::MotorPositions::SharedPtr msg);
  void velocityCommandCallback(rise_motion_messages::msg::MotorVelocity::SharedPtr msg);
  void torqueOffsetCallback(rise_motion_messages::msg::MotorTorqueOffset::SharedPtr msg);
  void publishFeedback();
  void publishFullFeedback();
  void enableServiceCallback(
    const std::shared_ptr<rise_motion_messages::srv::EnableEthercatSrv::Request> request,
    std::shared_ptr<rise_motion_messages::srv::EnableEthercatSrv::Response> response);
  void sdoReadServiceCallback(
      const std::shared_ptr<rise_motion_messages::srv::SDOReadSrv::Request>
	  request,
      std::shared_ptr<rise_motion_messages::srv::SDOReadSrv::Response>
	  response);
  void sdoWriteServiceCallback(
      const std::shared_ptr<rise_motion_messages::srv::SDOWriteSrv::Request>
	  request,
      std::shared_ptr<rise_motion_messages::srv::SDOWriteSrv::Response>
	  response);
  void setOperationModeCallback(
      std::shared_ptr<rise_motion_messages::srv::SetOperationModeSrv::Request> request,
      std::shared_ptr<rise_motion_messages::srv::SetOperationModeSrv::Response> response);
};
