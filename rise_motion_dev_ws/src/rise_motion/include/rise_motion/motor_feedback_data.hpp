#pragma once
#include <cstdint>

struct MotorFeedbackData {
  uint16_t statusword;
  int8_t   op_mode_display;
  int32_t  position;
  int32_t  velocity_value;
  int16_t  torque_value;
  uint16_t analog_input1;  // Kraftsensor Kanal 1 (ADC-Ticks, 0=Umin, 65535=Umax)
  uint16_t analog_input2;
  uint16_t analog_input3;
  uint16_t analog_input4;
  uint32_t tuning_status;
  uint32_t digital_inputs;
  uint32_t user_miso;
  uint32_t timestamp;
  int32_t  position_demand_internal_value;
  int32_t  velocity_demand_value;
  int16_t  torque_demand;
};
