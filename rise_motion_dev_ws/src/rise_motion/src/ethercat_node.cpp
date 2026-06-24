#include <cstdlib>
#include <functional>
#include <rclcpp/logging.hpp>
#include <rise_motion/ethercat_node.hpp>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;

EthercatNode::EthercatNode(ECManager &ec_manager)
    : Node("ethercat_node"), ec_manager_(ec_manager) {

  cmd_sub_ = create_subscription<rise_motion_messages::msg::MotorPositions>(
      "motor_commands", 10,
      std::bind(&EthercatNode::commandCallback, this, _1));

  feedback_pub_ = create_publisher<rise_motion_messages::msg::MotorPositions>(
      "motor_feedback", 10);

  feedback_timer_ =
      create_wall_timer(std::chrono::milliseconds(10),
			std::bind(&EthercatNode::publishFeedback, this));

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
	  std::make_unique<std::thread>(&ECManager::cyclic_loop, &ec_manager_);
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

  auto submission = ec_manager_.enqueue_sdo_read(request->device_id, request->index, request->subindex, request->value_size);

  if (submission.id == SdoScheduler::INVALID_JOB_ID) {
    RCLCPP_WARN(get_logger(), "ethercat_node: Could not queue SDO read request");
    response->status_code = 0;
    return;
  }

  if (submission.future.wait_for(SDO_SERVICE_TIMEOUT) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "SDO read request timed out");

    ec_manager_.cancel_sdo_request(submission.id);

    response->status_code = 0;
    return;
  }

  auto result = submission.future.get();

  if (!result) {
    RCLCPP_WARN(get_logger(), "SDO read failed: %s", result.error.message);

    response->status_code = 0;
    return;
  }

  response->status_code = 1;
  response->device_id = request->device_id;
  response->index = request->index;
  response->subindex = request->subindex;
  response->value = result.value;
  response->value_type = request->value_type;
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

  auto submission = ec_manager_.enqueue_sdo_write(request->device_id, request->index, request->subindex, request->value);

  if (submission.id == SdoScheduler::INVALID_JOB_ID) {
    RCLCPP_WARN(get_logger(), "Could not queue SDO write request");
    response->status_code = 0;
    return;
  }

  if (submission.future.wait_for(SDO_SERVICE_TIMEOUT) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "SDO write request timed out");

    ec_manager_.cancel_sdo_request(submission.id);

    response->status_code = 0;
    return;
  }

  auto result = submission.future.get();

  if (!result) {
    RCLCPP_WARN(get_logger(), "SDO write failed: %s", result.error.message);

    response->status_code = 0;
    return;
  }

  response->status_code = 1;
  response->device_id = request->device_id;
  response->index = request->index;
  response->subindex = request->subindex;
  response->value_type = request->value_type;
}
