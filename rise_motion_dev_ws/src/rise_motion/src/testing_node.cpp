#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rise_motion_messages/msg/motor_positions.hpp>
#include <rise_motion_messages/srv/enable_ethercat_srv.hpp>
#include <rise_motion_messages/srv/sdo_read_srv.hpp>
#include <rise_motion/sdo_serializer.hpp>


class TestNode : public rclcpp::Node {
public:
  TestNode(int incs) : Node("test_node"), valid_positions(false), increment(incs) {
    // Subscribe to the input topic
    RCLCPP_INFO(get_logger(), "Starting TestNode");
    input_sub =
        this->create_subscription<rise_motion_messages::msg::MotorPositions>(
            "motor_feedback", 10,
            [this](rise_motion_messages::msg::MotorPositions msg) {
              if (!valid_positions) {
                RCLCPP_INFO(get_logger(), "Got feedback");
                valid_positions = true;
              }
              motor_pos = msg.positions;
            });
    publish_timer_ = create_wall_timer(std::chrono::milliseconds(1), [this]() {
      if (!valid_positions)
        return;
      auto response = rise_motion_messages::msg::MotorPositions();
      response.positions.resize(motor_pos.size());
      for (size_t i = 0; i < motor_pos.size(); i++) {
        response.positions[i] = motor_pos[i] + increment;
      }

      output_pub->publish(response);
    });

    output_pub =
        this->create_publisher<rise_motion_messages::msg::MotorPositions>(
            "motor_commands", 10);

    client = this->create_client<rise_motion_messages::srv::EnableEthercatSrv>(
        "enable_ethercat");
  }
  ~TestNode() {
    RCLCPP_INFO(get_logger(), "Bye :)");
  }
  int request_enable_ethercat() {
    RCLCPP_INFO(get_logger(), "Incrementing motor position with %d", increment);
    RCLCPP_INFO(get_logger(), "Requesting Enable Ethercat");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "client interrupted while waiting for service to appear.");
        return 1;
      }
      RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }
    auto request = std::make_shared<
        rise_motion_messages::srv::EnableEthercatSrv::Request>();
    request->enable = true;
    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->shared_from_this(),
                                           result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "service call failed :(");
      client->remove_pending_request(result_future);
      return 1;
    }
    auto result = result_future.get();
    RCLCPP_INFO(get_logger(), "Done requesting");
    return result->status_enable;
  }

private:
  std::vector<int> motor_pos;
  void print_motor_positions(std::vector<int32_t> const &motor_pos,
                             std::string const &prefix) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < motor_pos.size(); ++i) {
      oss << motor_pos[i];
      if (i != motor_pos.size() - 1) {
        oss << ", ";
      }
    }
    oss << "]";

    RCLCPP_INFO(this->get_logger(), "%s: %s", prefix.c_str(),
                oss.str().c_str());
  }

  bool valid_positions;
  rclcpp::Subscription<rise_motion_messages::msg::MotorPositions>::SharedPtr
      input_sub;
  rclcpp::Publisher<rise_motion_messages::msg::MotorPositions>::SharedPtr
      output_pub;

  rclcpp::Client<rise_motion_messages::srv::EnableEthercatSrv>::SharedPtr
      client;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  int increment;
};

int main(int argc, char **argv) {
  int incs = 10;
  if (argc >= 2) {
    incs = atoi(argv[1]);
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestNode>(incs);

  while (!node->request_enable_ethercat()) {
  }
  
  rclcpp::Client<rise_motion_messages::srv::SDOReadSrv>::SharedPtr
      sdo_client;
  sdo_client = node->create_client<rise_motion_messages::srv::SDOReadSrv>(
      "sdo_read");
  auto request = std::make_shared<rise_motion_messages::srv::SDOReadSrv::Request>();
  request->device_id = 1;
  request->index = 0x1008;
  request->subindex = 0;
  request->value_type = 0;
  auto result = sdo_client->async_send_request(request);
  // wait for result
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sdo_value: %s", (std::string(sdo::deserialize<sdo::STRING<50>>(result.get()->value))).std::string::c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service sdo_read");
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
