#include <cstdlib>
#include <functional>
#include <rclcpp/logging.hpp>
#include <rise_motion/ethercat_node.hpp>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;

EthercatNode::EthercatNode(IECManager &ec_manager)
    : Node("ethercat_node"), ec_manager_(ec_manager) {

  cmd_sub_ = create_subscription<rise_motion_messages::msg::MotorPositions>(
      "motor_commands", 10,
      std::bind(&EthercatNode::commandCallback, this, _1));

  feedback_pub_ = create_publisher<rise_motion_messages::msg::MotorPositions>(
      "motor_feedback", 10);

  feedback_timer_ =
      create_wall_timer(std::chrono::milliseconds(10),
			std::bind(&EthercatNode::publishFeedback, this));

  full_feedback_pub_ =
      create_publisher<rise_motion_messages::msg::MotorFeedbackFull>(
          "motor_feedback_full", 10);

  full_feedback_timer_ =
      create_wall_timer(std::chrono::milliseconds(10),
                        std::bind(&EthercatNode::publishFullFeedback, this));

  enable_srv_ = create_service<rise_motion_messages::srv::EnableEthercatSrv>(
      "enable_ethercat",
      std::bind(&EthercatNode::enableServiceCallback, this, _1, _2));

  sdo_read_srv_ = create_service<rise_motion_messages::srv::SDOReadSrv>(
      "sdo_read",
      std::bind(&EthercatNode::sdoReadServiceCallback, this, _1, _2));

  sdo_write_srv_ = create_service<rise_motion_messages::srv::SDOWriteSrv>(
      "sdo_write",
      std::bind(&EthercatNode::sdoWriteServiceCallback, this, _1, _2));

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

/**
 * @brief ROS subscriber callback for motor commands
 *
 * Communication Thread (ROS side) - SEND PATH: ROS → EtherCAT
 *
 * Uses APSA's comm_write() for lock-free transfer to the 1kHz EtherCAT loop.
 * Never blocks the EtherCAT loop.
 */
void EthercatNode::commandCallback(
    const rise_motion_messages::msg::MotorPositions::SharedPtr msg) {
  std::vector<int32_t> positions(msg->positions.begin(), msg->positions.end());

  if (!ec_manager_.set_motor_values_apsa(positions)) {
    RCLCPP_WARN(get_logger(), "Failed to queue motor commands");
  }
}

/**
 * @brief ROS publisher timer callback for motor feedback
 *
 * Communication Thread (ROS side) - RECEIVE PATH: EtherCAT → ROS
 *
 * Uses APSA's comm_read() for lock-free transfer from the 1kHz EtherCAT loop.
 * Only publishes when NEW feedback is available.
 */
void EthercatNode::publishFeedback() {
  std::vector<int32_t> positions;

  if (ethercat_enabled_ && ec_manager_.get_motor_values_apsa(positions)) {
    auto msg = rise_motion_messages::msg::MotorPositions();
    msg.positions.assign(positions.begin(), positions.end());
    feedback_pub_->publish(msg);
  }
}

void EthercatNode::enableServiceCallback(
    const std::shared_ptr<rise_motion_messages::srv::EnableEthercatSrv::Request>
	request,
    std::shared_ptr<rise_motion_messages::srv::EnableEthercatSrv::Response>
	response) {
  if (request->enable && !ethercat_enabled_) {
    RCLCPP_INFO(get_logger(), "Enabling EtherCAT communication");
    if (ec_manager_.init_ec() == EXIT_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Couldn't init ethercat");
    } else {
      ec_thread_ =
	  std::make_unique<std::thread>([this]() { ec_manager_.cyclic_loop(); });
      ethercat_enabled_ = true;
    }
  } else if (!request->enable && ethercat_enabled_) {
    RCLCPP_INFO(get_logger(), "Disabling EtherCAT communication");
    ec_manager_.stop();
    if (ec_thread_ && ec_thread_->joinable()) {
      ec_thread_->join();
    }
    ethercat_enabled_ = false;
  }

  response->status_enable = ethercat_enabled_ ? 1 : 0;
}

void EthercatNode::sdoReadServiceCallback(
    const std::shared_ptr<rise_motion_messages::srv::SDOReadSrv::Request>
	request,
    std::shared_ptr<rise_motion_messages::srv::SDOReadSrv::Response> response) {

  if (!ethercat_enabled_ || !ec_manager_.is_running()) {
    response->status_code = 0;
    return;
  }

  std::vector<uint8_t> value;
  bool success = ec_manager_.sdo_read(request->device_id, request->index,
				      request->subindex, value);

    if (!success) {
    RCLCPP_WARN(get_logger(), "Read failed");
    response->status_code = 0;
    return;
  }

  response->status_code = 1;
  response->device_id	= request->device_id;
  response->index	= request->index;
  response->subindex	= request->subindex;
  response->value	= value;
  response->value_type	= 0;
}
void EthercatNode::publishFullFeedback() {
  std::vector<MotorFeedbackData> feedback;

  if (ethercat_enabled_ && ec_manager_.get_full_feedback_apsa(feedback)) {
    auto msg = rise_motion_messages::msg::MotorFeedbackFull();
    msg.header.stamp = now();
    for (const auto& f : feedback) {
      msg.statusword.push_back(f.statusword);
      msg.op_mode_display.push_back(f.op_mode_display);
      msg.positions.push_back(f.position);
      msg.velocity_value.push_back(f.velocity_value);
      msg.torque_value.push_back(f.torque_value);
      msg.analog_input1.push_back(f.analog_input1);
      msg.analog_input2.push_back(f.analog_input2);
      msg.analog_input3.push_back(f.analog_input3);
      msg.analog_input4.push_back(f.analog_input4);
      msg.tuning_status.push_back(f.tuning_status);
      msg.digital_inputs.push_back(f.digital_inputs);
      msg.user_miso.push_back(f.user_miso);
      msg.timestamp.push_back(f.timestamp);
      msg.position_demand_internal_value.push_back(f.position_demand_internal_value);
      msg.velocity_demand_value.push_back(f.velocity_demand_value);
      msg.torque_demand.push_back(f.torque_demand);
    }
    full_feedback_pub_->publish(msg);
  }
}

void EthercatNode::sdoWriteServiceCallback(
    const std::shared_ptr<rise_motion_messages::srv::SDOWriteSrv::Request>
	request,
    std::shared_ptr<rise_motion_messages::srv::SDOWriteSrv::Response>
	response) {
  if (!ethercat_enabled_ || !ec_manager_.is_running()) {
    response->status_code = 0;
    return;
  }
  RCLCPP_INFO(get_logger(), "Got sdo_write request");
  bool success = ec_manager_.sdo_write(request->device_id, request->index,
				       request->subindex, request->value);
  RCLCPP_INFO(get_logger(), "%d", success);
  if (!success) {
    RCLCPP_INFO(get_logger(), "Write failed");
    response->status_code = 0;
    return;
  }

  response->status_code = 1;
  response->device_id	= request->device_id;
  response->index	= request->index;
  response->subindex	= request->subindex;
  response->value_type	= 0;
}
