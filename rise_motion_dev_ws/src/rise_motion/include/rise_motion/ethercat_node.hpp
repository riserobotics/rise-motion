#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rise_motion/ec_manager.hpp>
#include <rise_motion_messages/msg/motor_positions.hpp>
#include <rise_motion_messages/srv/enable_ethercat_srv.hpp>
#include <rise_motion_messages/srv/sdo_read_srv.hpp>
#include <rise_motion_messages/srv/sdo_write_srv.hpp>
#include <rise_motion_messages/srv/foe_read_srv.hpp>
#include <rise_motion_messages/srv/foe_write_srv.hpp>
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
  rclcpp::Service<rise_motion_messages::srv::SDOReadSrv>::SharedPtr sdo_read_srv_;
  rclcpp::Service<rise_motion_messages::srv::SDOWriteSrv>::SharedPtr sdo_write_srv_;
  rclcpp::Service<rise_motion_messages::srv::FOEReadSrv>::SharedPtr foe_read_srv_;
  rclcpp::Service<rise_motion_messages::srv::FOEWriteSrv>::SharedPtr foe_write_srv_;


  void commandCallback(const rise_motion_messages::msg::MotorPositions::SharedPtr msg);
  void publishFeedback();
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
  void foeReadServiceCallback(
      const std::shared_ptr<rise_motion_messages::srv::FOEReadSrv::Request>
	  request,
      std::shared_ptr<rise_motion_messages::srv::FOEReadSrv::Response> 
    response);
  void foeWriteServiceCallback(
      const std::shared_ptr<rise_motion_messages::srv::FOEWriteSrv::Request>
	  request,
      std::shared_ptr<rise_motion_messages::srv::FOEWriteSrv::Response>
	  response);
};
