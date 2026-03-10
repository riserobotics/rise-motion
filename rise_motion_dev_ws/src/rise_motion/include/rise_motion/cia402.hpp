#pragma once
#include <array>
#include <cstdint>
#include <string>

#include <osal_defs.h>

OSAL_PACKED_BEGIN
typedef struct OSAL_PACKED {
  uint16_t Statusword;
  int8_t OpModeDisplay;
  int32_t PositionValue;
  int32_t VelocityValue;
  int16_t TorqueValue;
  uint16_t AnalogInput1;
  uint16_t AnalogInput2;
  uint16_t AnalogInput3;
  uint16_t AnalogInput4;
  uint32_t TuningStatus;
  uint32_t DigitalInputs;
  uint32_t UserMISO;
  uint32_t Timestamp;
  int32_t PositionDemandInternalValue;
  int32_t VelocityDemandValue;
  int16_t TorqueDemand;
} CiA402_Inputs;
OSAL_PACKED_END

OSAL_PACKED_BEGIN
typedef struct OSAL_PACKED {
  uint16_t Controlword;
  int8_t OpMode;
  int16_t TargetTorque;
  int32_t TargetPosition;
  int32_t TargetVelocity;
  int16_t TorqueOffset;
  uint32_t TuningCommand;
  uint32_t PhysicalOutputs;
  uint32_t BitMask;
  uint32_t UserMOSI;
  int32_t VelocityOffset;
} CiA402_Outputs;
OSAL_PACKED_END

class CiA402Motor {
public:
  enum class State {
    NOT_READY_TO_SWITCH_ON,
    SWITCH_ON_DISABLED,
    READY_TO_SWITCH_ON,
    SWITCHED_ON,
    OPERATION_ENABLED,
    QUICK_STOP_ACTIVE,
    FAULT_REACTION_ACTIVE,
    FAULT,
    UNKNOWN
  };

  enum class Operation {
    SHUTDOWN,
    SWITCH_ON,
    DISABLE_VOLTAGE,
    QUICK_STOP,
    DISABLE_OPERATION,
    ENABLE_OPERATION,
    FAULT_RESET,
    NO_OP
  };

  // https://doc.synapticon.com/circulo/sw5.4/objects_html/6xxx/6060.html
  enum class ModeOfOperation : int8_t {
    ImpedanceMode = -6,
    JointTorqueMode = -5,
    SystemIdentificationMode = -4,
    OpenLoopFieldMode = -3,
    DiagnosticsMode = -2,
    CoggingCompensationRecordingMode = -1,
    ProfilePositionMode = 1,
    ProfileVelocityMode = 3,
    TorqueProfileMode = 4,
    HomingMode = 6,
    CyclicSyncPositionMode = 8,
    CyclicSyncVelocityMode = 9,
    CyclicSyncTorqueMode = 10
  };

  CiA402Motor(CiA402_Inputs *inputs, CiA402_Outputs *outputs);

  /**
   * @brief Gets the motor's current state based on its status word.
   *
   * Compares the status word against predefined patterns. Returns the matching
   * state, or `State::UNKNOWN` if no match is found.
   *
   * @return The motor's state if a match is found; otherwise, `State::UNKNOWN`.
   *
   * @warning Returning `std::nullopt` indicates an unrecognized status word.
   *          Callers must handle this case to avoid undefined behavior.
   *
   * @see CiA402Motor::State, state_patterns
   */
  State get_state() const;
  std::string state_as_string() const;
  bool is_state(State s) const;
  bool is_operation_enabled() const;
  bool is_fault() const;

  void to_operation_enabled();
  void to_switch_on_disabled();
  void transition_to(State s);
  void reset_fault();
  void set_control_word(Operation op);
  void set_mode_of_operation(ModeOfOperation m);

  CiA402_Inputs *inputs;
  CiA402_Outputs *outputs;

private:
  // origin destination
  constexpr static std::array<std::array<Operation, 8>, 8> transition_table{
      {// Current State: NOT_READY_TO_SWITCH_ON (0)
       {{
           Operation::NO_OP, // NOT_READY_TO_SWITCH_ON
           Operation::NO_OP, // SWITCH_ON_DISABLED
           Operation::NO_OP, // READY_TO_SWITCH_ON
           Operation::NO_OP, // SWITCHED_ON
           Operation::NO_OP, // OPERATION_ENABLED
           Operation::NO_OP, // QUICK_STOP_ACTIVE
           Operation::NO_OP, // FAULT_REACTION_ACTIVE
           Operation::NO_OP  // FAULT
       }},
       // Current State: SWITCH_ON_DISABLED (1)
       {{
           Operation::NO_OP,    // NOT_READY_TO_SWITCH_ON
           Operation::NO_OP,    // SWITCH_ON_DISABLED
           Operation::SHUTDOWN, // READY_TO_SWITCH_ON
           Operation::SHUTDOWN, // SWITCHED_ON
           Operation::SHUTDOWN, // OPERATION_ENABLED
           Operation::SHUTDOWN, // QUICK_STOP_ACTIVE
           Operation::NO_OP,    // FAULT_REACTION_ACTIVE
           Operation::NO_OP     // FAULT
       }},
       // Current State: READY_TO_SWITCH_ON (2)
       {{
           Operation::NO_OP,           // NOT_READY_TO_SWITCH_ON
           Operation::DISABLE_VOLTAGE, // SWITCH_ON_DISABLED
           Operation::NO_OP,           // READY_TO_SWITCH_ON
           Operation::SWITCH_ON,       // SWITCHED_ON
           Operation::SWITCH_ON,       // OPERATION_ENABLED
           Operation::SWITCH_ON,       // QUICK_STOP_ACTIVE
           Operation::NO_OP,           // FAULT_REACTION_ACTIVE
           Operation::NO_OP            // FAULT
       }},
       // Current State: SWITCHED_ON (3)
       {{
           Operation::NO_OP,            // NOT_READY_TO_SWITCH_ON
           Operation::DISABLE_VOLTAGE,  // SWITCH_ON_DISABLED
           Operation::SHUTDOWN,         // READY_TO_SWITCH_ON
           Operation::NO_OP,            // SWITCHED_ON
           Operation::ENABLE_OPERATION, // OPERATION_ENABLED
           Operation::ENABLE_OPERATION, // QUICK_STOP_ACTIVE
           Operation::NO_OP,            // FAULT_REACTION_ACTIVE
           Operation::NO_OP             // FAULT
       }},
       // Current State: OPERATION_ENABLED (4)
       {{
           Operation::NO_OP,             // NOT_READY_TO_SWITCH_ON
           Operation::DISABLE_VOLTAGE,   // SWITCH_ON_DISABLED
           Operation::SHUTDOWN,          // READY_TO_SWITCH_ON
           Operation::DISABLE_OPERATION, // SWITCHED_ON
           Operation::NO_OP,             // OPERATION_ENABLED
           Operation::QUICK_STOP,        // QUICK_STOP_ACTIVE
           Operation::NO_OP,             // FAULT_REACTION_ACTIVE
           Operation::NO_OP              // FAULT
       }},
       // Current State: QUICK_STOP_ACTIVE (5)
       {{
           Operation::NO_OP,            // NOT_READY_TO_SWITCH_ON
           Operation::DISABLE_VOLTAGE,  // SWITCH_ON_DISABLED
           Operation::ENABLE_OPERATION, // READY_TO_SWITCH_ON
           Operation::ENABLE_OPERATION, // SWITCHED_ON
           Operation::ENABLE_OPERATION, // OPERATION_ENABLED
           Operation::NO_OP,            // QUICK_STOP_ACTIVE
           Operation::NO_OP,            // FAULT_REACTION_ACTIVE
           Operation::NO_OP             // FAULT
       }},
       // Current State: FAULT_REACTION_ACTIVE (6)
       {{
           Operation::NO_OP, // NOT_READY_TO_SWITCH_ON
           Operation::NO_OP, // SWITCH_ON_DISABLED
           Operation::NO_OP, // READY_TO_SWITCH_ON
           Operation::NO_OP, // SWITCHED_ON
           Operation::NO_OP, // OPERATION_ENABLED
           Operation::NO_OP, // QUICK_STOP_ACTIVE
           Operation::NO_OP, // FAULT_REACTION_ACTIVE
           Operation::NO_OP  // FAULT
       }},
       // Current State: FAULT (7)
       {{
           Operation::NO_OP, // NOT_READY_TO_SWITCH_ON
           Operation::NO_OP, // SWITCH_ON_DISABLED
           Operation::NO_OP, // READY_TO_SWITCH_ON
           Operation::NO_OP, // SWITCHED_ON
           Operation::NO_OP, // OPERATION_ENABLED
           Operation::NO_OP, // QUICK_STOP_ACTIVE
           Operation::NO_OP, // FAULT_REACTION_ACTIVE
           Operation::NO_OP  // FAULT
       }}}};
  struct StatePattern {
    uint16_t mask;
    uint16_t value;
    State state;
  };

  static constexpr StatePattern state_patterns[] = {
      // https://doc.synapticon.com/circulo/system_integration/status_and_controlword.html
      // mask     value      state
      {0b1001111, 0b0000000, State::NOT_READY_TO_SWITCH_ON},
      {0b1001111, 0b1000000, State::SWITCH_ON_DISABLED},
      {0b1101111, 0b0100001, State::READY_TO_SWITCH_ON},
      {0b1101111, 0b0100011, State::SWITCHED_ON},
      {0b1101111, 0b0100111, State::OPERATION_ENABLED},
      {0b1101111, 0b0000111, State::QUICK_STOP_ACTIVE},
      {0b1001111, 0b0001111, State::FAULT_REACTION_ACTIVE},
      {0b1001111, 0b0001000, State::FAULT}};

  struct ControlPattern {
    uint16_t mask;
    uint16_t value;
    Operation op;
  };

  static constexpr ControlPattern control_patterns[] = {
      // https://doc.synapticon.com/circulo/system_integration/status_and_controlword.html
      // mask      value       operation
      {0b10000111, 0b00000110, Operation::SHUTDOWN},
      {0b10001111, 0b00000111, Operation::SWITCH_ON},
      {0b10000010, 0b00000000, Operation::DISABLE_VOLTAGE},
      {0b10000110, 0b00000010, Operation::QUICK_STOP},
      {0b10001111, 0b00000111, Operation::DISABLE_OPERATION},
      {0b10001111, 0b00001111, Operation::ENABLE_OPERATION},
      {0b10000000, 0b10000000, Operation::FAULT_RESET}};
};
