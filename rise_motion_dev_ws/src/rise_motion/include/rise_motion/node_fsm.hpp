#pragma once
#include <map>
#include <string>
#include <utility>
class StateMachine {
public:
  enum class State { Idle, Initialized, Operational, Error };
  enum class Event { Idle, Initialized, Operational, Error };

  StateMachine();

  State handle_event(Event e);
  void set_state(State s);
  State get_state();
  static std::string state_to_string(State s);

private:
  State state;
  static std::map<std::pair<State, Event>, State> transition_table;
};
