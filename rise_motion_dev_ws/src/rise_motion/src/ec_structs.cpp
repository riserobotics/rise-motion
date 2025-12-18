#include <rise_motion/ec_structs.hpp>
#include <stdio.h>
void print_motor_outputs(const MotorOutputs *o) {
  printf(
    "MotorOutputs:\n"
    "  Statusword                 : 0x%04X\n"
    "  OpModeDisplay              : %d\n"
    "  PositionValue              : %d\n"
    "  VelocityValue              : %d\n"
    "  TorqueValue                : %d\n"
    "  AnalogInput1               : %u\n"
    "  AnalogInput2               : %u\n"
    "  AnalogInput3               : %u\n"
    "  AnalogInput4               : %u\n"
    "  TuningStatus               : 0x%08X\n"
    "  DigitalInputs              : 0x%08X\n"
    "  UserMISO                   : 0x%08X\n"
    "  Timestamp                  : %u\n"
    "  PositionDemandInternal     : %d\n"
    "  VelocityDemandValue        : %d\n"
    "  TorqueDemand               : %d\n",
    o->Statusword,
    o->OpModeDisplay,
    o->PositionValue,
    o->VelocityValue,
    o->TorqueValue,
    o->AnalogInput1,
    o->AnalogInput2,
    o->AnalogInput3,
    o->AnalogInput4,
    o->TuningStatus,
    o->DigitalInputs,
    o->UserMISO,
    o->Timestamp,
    o->PositionDemandInternalValue,
    o->VelocityDemandValue,
    o->TorqueDemand
  );
}

void print_motor_inputs(const MotorInputs *i) {
  printf(
    "MotorInputs:\n"
    "  Controlword      : 0x%04X\n"
    "  OpMode           : %d\n"
    "  TargetTorque     : %d\n"
    "  TargetPosition   : %d\n"
    "  TargetVelocity   : %d\n"
    "  TorqueOffset     : %d\n"
    "  TuningCommand    : 0x%08X\n"
    "  PhysicalOutputs  : 0x%08X\n"
    "  BitMask          : 0x%08X\n"
    "  UserMOSI         : 0x%08X\n"
    "  VelocityOffset   : %d\n",
    i->Controlword,
    i->OpMode,
    i->TargetTorque,
    i->TargetPosition,
    i->TargetVelocity,
    i->TorqueOffset,
    i->TuningCommand,
    i->PhysicalOutputs,
    i->BitMask,
    i->UserMOSI,
    i->VelocityOffset
  );
}
