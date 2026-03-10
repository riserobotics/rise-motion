#include <rise_motion/cia402.hpp>

CiA402Motor::CiA402Motor(CiA402_Inputs *inputs, CiA402_Outputs *outputs)
    : inputs(inputs), outputs(outputs) {}

CiA402Motor::State CiA402Motor::get_state() const {
  const uint16_t sw = inputs->Statusword;
  for (const auto &p : state_patterns) {
    if ((sw & p.mask) == p.value)
      return p.state;
  }
  return State::UNKNOWN;
}

bool CiA402Motor::is_state(State s) const {
  State actual_state = get_state();
  return (actual_state == s);
}

void CiA402Motor::transition_to(State destination) {
  State origin = get_state();
  if (origin != State::UNKNOWN) {
    Operation op = transition_table[static_cast<int>(origin)]
                                   [static_cast<int>(destination)];
    if (op != Operation::NO_OP) {
      set_control_word(op);
    }
  }
}

bool CiA402Motor::is_operation_enabled() const {
  return is_state(State::OPERATION_ENABLED);
}

bool CiA402Motor::is_fault() const { return is_state(State::FAULT); }

void CiA402Motor::reset_fault() { set_control_word(Operation::FAULT_RESET); }

void CiA402Motor::set_control_word(Operation op) {
  for (auto &p : control_patterns) {
    if (p.op == op) {
      outputs->Controlword = (outputs->Controlword & ~p.mask) | p.value;
      break;
    }
  }
}

void CiA402Motor::set_mode_of_operation(CiA402Motor::ModeOfOperation m) {
  outputs->OpMode = static_cast<int8_t>(m);
}

std::string CiA402Motor::state_as_string() const {
  State s = get_state();
  switch (s) {
  case State::NOT_READY_TO_SWITCH_ON:
    return "NOT_READY_TO_SWITCH_ON";
  case State::SWITCH_ON_DISABLED:
    return "SWITCH_ON_DISABLED";
  case State::READY_TO_SWITCH_ON:
    return "READY_TO_SWITCH_ON";
  case State::SWITCHED_ON:
    return "SWITCHED_ON";
  case State::OPERATION_ENABLED:
    return "OPERATION_ENABLED";
  case State::QUICK_STOP_ACTIVE:
    return "QUICK_STOP_ACTIVE";
  case State::FAULT_REACTION_ACTIVE:
    return "FAULT_REACTION_ACTIVE";
  case State::FAULT:
    return "FAULT";
  case State::UNKNOWN:
    return "UNKNOWN";
  }
}
