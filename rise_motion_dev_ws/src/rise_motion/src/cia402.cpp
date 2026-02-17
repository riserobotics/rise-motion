#include <cstdint>
#include <optional>
#include <rise_motion/cia402.hpp>

CiA402Motor::CiA402Motor(CiA402_Inputs *inputs, CiA402_Outputs *outputs)
    : inputs(inputs), outputs(outputs) {}

std::optional<CiA402Motor::State> CiA402Motor::get_state() const {
  const uint16_t sw = inputs->Statusword;
  for (const auto &p : state_patterns) {
    if ((sw & p.mask) == p.value)
      return p.state;
  }
  return std::nullopt;
}

bool CiA402Motor::is_state(State s) const {
  std::optional<State> actual_state = get_state();
  if (actual_state.has_value() && actual_state.value() == s) {
    return true;
  } else {
    return false;
  }
}

bool CiA402Motor::is_operation_enabled() const {
  return is_state(State::OPERATION_ENABLED);
}

void CiA402Motor::to_operation_enabled() {
  std::optional<State> s = get_state();

  if (!s.has_value()) {
    return;
  }

  std::optional<Operation> next_op = std::nullopt;
  switch (s.value()) {
  case State::SWITCH_ON_DISABLED:
    next_op = Operation::SHUTDOWN;
    break;
  case State::READY_TO_SWITCH_ON:
    next_op = Operation::SWITCH_ON;
    break;
  case State::SWITCHED_ON:
    next_op = Operation::ENABLE_OPERATION;
    break;
  case State::OPERATION_ENABLED:
  case State::QUICK_STOP_ACTIVE:
  case State::FAULT_REACTION_ACTIVE:
  case State::FAULT:
  default:
    return;
  }

  if (next_op.has_value()) {
    set_control_word(next_op.value());
  }
}

void CiA402Motor::to_switch_on_disabled() {
  std::optional<State> s = get_state();

  if (!s.has_value()) {
    return;
  }

  std::optional<Operation> next_op = std::nullopt;
  switch (s.value()) {
  case State::OPERATION_ENABLED:
  case State::QUICK_STOP_ACTIVE:
    next_op = Operation::SHUTDOWN;
  case State::FAULT_REACTION_ACTIVE:
  case State::FAULT:
  case State::SWITCH_ON_DISABLED:
  case State::READY_TO_SWITCH_ON:
  case State::SWITCHED_ON:
  default:
    return;
  }

  if (next_op.has_value()) {
    set_control_word(next_op.value());
  }
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
  std::optional<State> s = get_state();
  if (s == std::nullopt) {
    return "UNKNOWN";
  }

  switch (s.value()) {
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
  }
  return "UNKNOWN";
}
