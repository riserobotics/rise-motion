#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <osal_defs.h>

namespace pdoMap{

  using PDOMappingEntry = std::vector<std::uint8_t>;

  inline PDOMappingEntry u8(std::uint8_t value) {
    return {value};
  }

  inline PDOMappingEntry u16(std::uint16_t value) {
    return {
        static_cast<std::uint8_t>(value & 0xFF),
        static_cast<std::uint8_t>((value >> 8) & 0xFF),
    };
  }

  inline PDOMappingEntry u32(std::uint32_t value) {
    return {
        static_cast<std::uint8_t>(value & 0xFF),
        static_cast<std::uint8_t>((value >> 8) & 0xFF),
        static_cast<std::uint8_t>((value >> 16) & 0xFF),
        static_cast<std::uint8_t>((value >> 24) & 0xFF),
    };
  }


  inline const std::vector<PDOMappingEntry> RX_PDO_1600 = {
    u32(0x60400010), // Controlword
    u32(0x60600008), // Modes of operation
    u32(0x60710010), // Target torque
    u32(0x607A0020), // Target position
    u32(0x60FF0020), // Target velocity
    u32(0x60B20010), // Torque offset
    u32(0x27010020), // Tuning command
  };

  inline const std::vector<PDOMappingEntry> RX_PDO_1601 = {
      u32(0x60FE0120), // Physical outputs
      u32(0x60FE0220), // Bit mask
  };

  inline const std::vector<PDOMappingEntry> RX_PDO_1602 = {
      u32(0x27030020), // User MOSI
      u32(0x60B10020), // Velocity offset
  };


  inline const std::vector<PDOMappingEntry> TX_PDO_1A00 = {
      u32(0x60410010), // Statusword
      u32(0x60610008), // Modes of operation display
      u32(0x60640020), // Position actual
      u32(0x606C0020), // Velocity actual
      u32(0x60770010), // Torque actual
  };

  inline const std::vector<PDOMappingEntry> TX_PDO_1A01 = {
      u32(0x24010010), // Analog input 1
      u32(0x24020010), // Analog input 2
      u32(0x24030010), // Analog input 3
      u32(0x24040010), // Analog input 4
      u32(0x27020020), // Tuning status
  };

  inline const std::vector<PDOMappingEntry> TX_PDO_1A02 = {
      u32(0x60FD0020), // Digital inputs
  };

  inline const std::vector<PDOMappingEntry> TX_PDO_1A03 = {
      u32(0x27040020), // User MISO
      u32(0x20F00020), // Timestamp
      u32(0x60FC0020), // Position demand internal
      u32(0x606B0020), // Velocity demand
      u32(0x60740010), // Torque demand
  };


  inline const std::vector<PDOMappingEntry> RX_ASSIGNMENT = {
      u16(0x1600),
      u16(0x1601),
      u16(0x1602),
  };

  inline const std::vector<PDOMappingEntry> TX_ASSIGNMENT = {
      u16(0x1A00),
      u16(0x1A01),
      u16(0x1A02),
      u16(0x1A03),
  };
}


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
  };

  enum class Operation {
    SHUTDOWN,
    SWITCH_ON,
    DISABLE_VOLTAGE,
    QUICK_STOP,
    DISABLE_OPERATION,
    ENABLE_OPERATION,
    FAULT_RESET
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
   * state, or `std::nullopt` if no match is found.
   *
   * @return The motor's state if a match is found; otherwise, `std::nullopt`.
   *
   * @warning Returning `std::nullopt` indicates an unrecognized status word.
   *          Callers must handle this case to avoid undefined behavior.
   *
   * @see CiA402Motor::State, state_patterns
   */
  std::optional<State> get_state() const;
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
