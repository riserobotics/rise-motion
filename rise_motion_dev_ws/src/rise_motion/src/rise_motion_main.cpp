#include <chrono>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rise_motion/node_fsm.hpp>
#include <rise_motion_messages/msg/state_current_msg.hpp>
#include <string>

class RiseMotionMain : public rclcpp::Node {
public:
  RiseMotionMain() : Node("rise_motion_main"), sm_() {
    publisher_ =
        this->create_publisher<rise_motion_messages::msg::StateCurrentMsg>(
            "rise_motion_state_changes", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                     [this]() -> void { this->on_time_out(); });
  }

private:
  void on_time_out() {
    auto message = rise_motion_messages::msg::StateCurrentMsg();
    StateMachine::State previous_state = sm_.get_state();
    StateMachine::State new_state;

    switch (previous_state) {
    case StateMachine::State::Idle:
      new_state = sm_.handle_event(StateMachine::Event::Initialized);
      break;
    case StateMachine::State::Initialized:
      new_state = sm_.handle_event(StateMachine::Event::Operational);
      break;
    case StateMachine::State::Operational:
      new_state = sm_.handle_event(StateMachine::Event::Error);
      break;
    case StateMachine::State::Error:
    default:
      new_state = sm_.handle_event(StateMachine::Event::Idle);
      break;
    }

    if (sm_.get_state() != previous_state) {
      message.previous_state = static_cast<uint8_t>(previous_state);
      message.current_state = static_cast<uint8_t>(new_state);
      sm_.set_state(new_state);

      RCLCPP_INFO(this->get_logger(), "State changed to '%s'",
                  StateMachine::state_to_string(sm_.get_state()).c_str());
      this->publisher_->publish(message);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rise_motion_messages::msg::StateCurrentMsg>::SharedPtr
      publisher_;
  StateMachine sm_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RiseMotionMain>());
  rclcpp::shutdown();
  return 0;
}
