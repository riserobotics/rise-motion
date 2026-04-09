# All CiA402_Inputs PDO fields as parallel arrays, index = motor
std_msgs/Header header  # ROS timestamp when message was published

uint16[] statusword
int8[]   op_mode_display
int32[]  positions
int32[]  velocity_value
int16[]  torque_value
uint16[] analog_input1  # Force sensor channel 1 (ADC ticks, 0=Umin, 65535=Umax)
uint16[] analog_input2
uint16[] analog_input3
uint16[] analog_input4
uint32[] tuning_status
uint32[] digital_inputs
uint32[] user_miso
uint32[] timestamp
int32[]  position_demand_internal_value
int32[]  velocity_demand_value
int16[]  torque_demand
