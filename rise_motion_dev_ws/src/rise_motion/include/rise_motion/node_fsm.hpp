#pragma once
#include <map>
#include <string>
#include <utility>
class StateMachine {
public:
  enum class State { Idle, Initialized, Operational, Error };
  enum class Event { ToIdle, ToInitialized, ToOperational, ToError };

  StateMachine();

  State handle_event(Event e);
  void set_state(State s);
  State get_state();
  static std::string state_to_string(State s);
  static std::string event_to_string(Event e);

private:
  State state;
  static std::map<std::pair<State, Event>, State> transition_table;
};
