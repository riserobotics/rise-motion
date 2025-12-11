#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rise_motion_messages/msg/motor_cmd_msg.hpp>
#include <rise_motion_messages/msg/state_current_msg.hpp>
#include <rise_motion_messages/srv/enable_ethercat_srv.hpp>
#include <rise_motion_messages/srv/get_ethercat_parameters_srv.hpp>
#include <rise_motion_messages/srv/set_ethercat_parameters_srv.hpp>

class RiseMotionMain : public rclcpp::Node {
public:
  RiseMotionMain() : Node("rise_motion_main") {
    setup_publishers();
    setup_services();
    setup_subscribers();
  }

private:
  void setup_publishers() {
    output_motor_cmd_publisher =
        this->create_publisher<rise_motion_messages::msg::MotorCmdMsg>(
            "output_motor_cmd", 10);
    rise_motion_state_changes_publisher =
        this->create_publisher<rise_motion_messages::msg::StateCurrentMsg>(
            "rise_motion_state_changes", 10);
  }
  void setup_services() {
    enable_ethercat_srv =
        this->create_service<rise_motion_messages::srv::EnableEthercatSrv>(
            "enable_ethercat",
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<
                       rise_motion_messages::srv::EnableEthercatSrv::Request>
                       request,
                   const std::shared_ptr<
                       rise_motion_messages::srv::EnableEthercatSrv::Response>
                       response) {
              this->handle_enable_ethercat(request_header, request, response);
            });

    get_ethercat_parameters_srv = this->create_service<
        rise_motion_messages::srv::GetEthercatParametersSrv>(
        "get_ethercat_parameters",
        [this](
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<
                rise_motion_messages::srv::GetEthercatParametersSrv::Request>
                request,
            const std::shared_ptr<
                rise_motion_messages::srv::GetEthercatParametersSrv::Response>
                response) {
          this->handle_get_ethercat_parameters(request_header, request,
                                               response);
        });

    set_ethercat_parameters_srv = this->create_service<
        rise_motion_messages::srv::SetEthercatParametersSrv>(
        "set_ethercat_parameters",
        [this](
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<
                rise_motion_messages::srv::SetEthercatParametersSrv::Request>
                request,
            const std::shared_ptr<
                rise_motion_messages::srv::SetEthercatParametersSrv::Response>
                response) {
          this->handle_set_ethercat_parameters(request_header, request,
                                               response);
        });
  }

  void setup_subscribers() {
    input_motor_cmd_subscriber =
        this->create_subscription<rise_motion_messages::msg::MotorCmdMsg>(
            "input_motor_cmd", 10,
            [this](rise_motion_messages::msg::MotorCmdMsg::SharedPtr msg) {
              handle_input_motor_cmd(msg);
            });
  }

  void handle_enable_ethercat(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<
          rise_motion_messages::srv::EnableEthercatSrv::Request>
          request,
      const std::shared_ptr<
          rise_motion_messages::srv::EnableEthercatSrv::Response>
          response) {
    (void)request_header;
    (void)request;
    (void)response;
    RCLCPP_WARN(get_logger(), "handle_enable_ethercat not implemented");
  }

  void handle_set_ethercat_parameters(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<
          rise_motion_messages::srv::SetEthercatParametersSrv::Request>
          request,
      const std::shared_ptr<
          rise_motion_messages::srv::SetEthercatParametersSrv::Response>
          response) {
    (void)request_header;
    (void)request;
    (void)response;
    RCLCPP_WARN(get_logger(), "handle_set_ethercat_parameters not implemented");
  }

  void handle_get_ethercat_parameters(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<
          rise_motion_messages::srv::GetEthercatParametersSrv::Request>
          request,
      const std::shared_ptr<
          rise_motion_messages::srv::GetEthercatParametersSrv::Response>
          response) {
    (void)request_header;
    (void)request;
    (void)response;
    RCLCPP_WARN(get_logger(), "handle_get_ethercat_parameters not implemented");
  }

  void handle_input_motor_cmd(
      const rise_motion_messages::msg::MotorCmdMsg::SharedPtr msg) {
    (void)msg;
    RCLCPP_WARN(get_logger(), "handle_input_motor_cmd not implemented");
  }

  rclcpp::Publisher<rise_motion_messages::msg::StateCurrentMsg>::SharedPtr
      rise_motion_state_changes_publisher;
  rclcpp::Publisher<rise_motion_messages::msg::MotorCmdMsg>::SharedPtr
      output_motor_cmd_publisher;

  rclcpp::Service<rise_motion_messages::srv::EnableEthercatSrv>::SharedPtr
      enable_ethercat_srv;
  rclcpp::Service<rise_motion_messages::srv::SetEthercatParametersSrv>::
      SharedPtr set_ethercat_parameters_srv;
  rclcpp::Service<rise_motion_messages::srv::GetEthercatParametersSrv>::
      SharedPtr get_ethercat_parameters_srv;

  rclcpp::Subscription<rise_motion_messages::msg::MotorCmdMsg>::SharedPtr
      input_motor_cmd_subscriber;

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = std::make_shared<RiseMotionMain>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
