#include <map>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rise_motion/node_fsm.hpp>

StateMachine::StateMachine() : state(State::Idle) {}

StateMachine::State StateMachine::handle_event(Event e) {
  std::pair<StateMachine::State, StateMachine::Event> keypair = {this->state,
                                                                 e};
  auto it = StateMachine::transition_table.find(keypair);
  if (it != StateMachine::transition_table.end()) {
    State old = state;
    this->state = it->second;
    RCLCPP_INFO(logger,
                "Transition %s --(%s)--> %s", state_to_string(old).c_str(),
                event_to_string(e).c_str(), state_to_string(state).c_str());
    return it->second;
  } else {
    RCLCPP_WARN(logger,
                "Invalid transition: (State: %s, Event: %s)",
                state_to_string(state).c_str(), event_to_string(e).c_str());
    return this->state;
  }
}

void StateMachine::set_state(State s) { state = s; }

StateMachine::State StateMachine::get_state() { return state; }

std::string StateMachine::state_to_string(State s) {
  switch (s) {
  case State::Idle:
    return "Idle";
  case State::Initialized:
    return "Initialized";
  case State::Operational:
    return "Operational";
  case State::Error:
    return "Error";
  }
  return "Unknown";
}

std::string StateMachine::event_to_string(Event e) {
  switch (e) {
  case Event::ToIdle:
    return "ToIdle";
  case Event::ToInitialized:
    return "ToInitialized";
  case Event::ToOperational:
    return "ToOperational";
  case Event::ToError:
    return "ToError";
  }
  return "Unknown";
}

rclcpp::Logger StateMachine::logger = rclcpp::get_logger("StateMachine");

std::map<std::pair<StateMachine::State, StateMachine::Event>,
         StateMachine::State>
    StateMachine::transition_table = {
        {{State::Idle, Event::ToInitialized}, State::Initialized},
        {{State::Idle, Event::ToError}, State::Initialized},
        {{State::Initialized, Event::ToOperational}, State::Operational},
        {{State::Initialized, Event::ToError}, State::Error},
        {{State::Operational, Event::ToError}, State::Error},
        {{State::Error, Event::ToIdle}, State::Idle}};
