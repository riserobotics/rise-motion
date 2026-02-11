#include <soem/soem.h>

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
} outputs;
OSAL_PACKED_END
OSAL_PACKED_BEGIN
typedef struct OSAL_PACKED {
  uint16_t Controlword;
  int8_t OpMode;
  int16_t TargetTorque;
  int32_t TargetPosition;
  int32_t TargetVelocity;
  int16_t TorqueOffset;
  int32_t TuningCommand;
  int32_t PhysicalOutputs;
  int32_t BitMask;
  int32_t UserMOSI;
  int32_t VelocityOffset;
} inputs;
OSAL_PACKED_END
