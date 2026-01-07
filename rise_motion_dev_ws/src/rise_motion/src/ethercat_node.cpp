#include <rise_motion/ethercat_node.hpp>
#include <functional>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;

EthercatNode::EthercatNode(ECManager& ec_manager)
  : Node("ethercat_node"), ec_manager_(ec_manager) {

  cmd_sub_ = create_subscription<rise_motion_messages::msg::MotorPositions>(
    "motor_commands", 10,
    std::bind(&EthercatNode::commandCallback, this, _1));

  feedback_pub_ = create_publisher<rise_motion_messages::msg::MotorPositions>(
    "motor_feedback", 10);

  feedback_timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&EthercatNode::publishFeedback, this));

  enable_srv_ = create_service<rise_motion_messages::srv::EnableEthercatSrv>(
    "enable_ethercat",
    std::bind(&EthercatNode::enableServiceCallback, this, std::placeholders::_1, std::placeholders::_2));


  RCLCPP_INFO(get_logger(), "EtherCAT node initialized");
}

EthercatNode::~EthercatNode() {
  if (ethercat_enabled_) {
    ec_manager_.stop();
    if (ec_thread_ && ec_thread_->joinable()) {
      ec_thread_->join();
    }
  }
}

void EthercatNode::commandCallback(
    const rise_motion_messages::msg::MotorPositions::SharedPtr msg) {
  std::vector<int32_t> positions(msg->positions.begin(), msg->positions.end());
  ec_manager_.set_motor_values(positions);
}

void EthercatNode::publishFeedback() {
  std::vector<int32_t> positions;
  positions.resize(6);
  ec_manager_.get_motor_values(positions);

  auto msg = rise_motion_messages::msg::MotorPositions();
  msg.positions.assign(positions.begin(), positions.end());
  feedback_pub_->publish(msg);
}

void EthercatNode::enableServiceCallback(
  const std::shared_ptr<rise_motion_messages::srv::EnableEthercatSrv::Request> request,
  std::shared_ptr<rise_motion_messages::srv::EnableEthercatSrv::Response> response)
{
  if (request->enable && !ethercat_enabled_) {
    RCLCPP_INFO(get_logger(), "Enabling EtherCAT communication");
    ec_manager_.init_ec();
    ec_thread_ = std::make_unique<std::thread>(&ECManager::cyclic_loop, &ec_manager_);
    ethercat_enabled_ = true;
  }
  else if (!request->enable && ethercat_enabled_) {
    RCLCPP_INFO(get_logger(), "Disabling EtherCAT communication");
    ec_manager_.stop();
    if (ec_thread_ && ec_thread_->joinable()) {
      ec_thread_->join();
    }
    ethercat_enabled_ = false;
  }

  response->status_enable = ethercat_enabled_ ? 1 : 0;
}

