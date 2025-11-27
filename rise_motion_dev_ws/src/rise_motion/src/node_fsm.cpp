#include <map>
#include <utility>

#include <rise_motion/node_fsm.hpp>

StateMachine::StateMachine() : state(State::Idle) {}

StateMachine::State StateMachine::handle_event(StateMachine::Event e) {
  std::pair<StateMachine::State, StateMachine::Event> keypair = {this->state,
                                                                 e};
  auto it = StateMachine::transition_table.find(keypair);
  if (it != StateMachine::transition_table.end()) {
    this->state = it->second;
    return it->second;
  } else {
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

std::map<std::pair<StateMachine::State, StateMachine::Event>,
         StateMachine::State>
    StateMachine::transition_table = {
        {{State::Idle, Event::Initialized}, State::Initialized},
        {{State::Initialized, Event::Operational}, State::Operational},
        {{State::Initialized, Event::Error}, State::Error},
        {{State::Operational, Event::Error}, State::Error},
        {{State::Error, Event::Idle}, State::Idle}};
