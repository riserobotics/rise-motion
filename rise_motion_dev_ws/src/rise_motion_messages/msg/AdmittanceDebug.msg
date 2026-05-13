# Per-motor debug data for admittance controller
# Arrays are indexed by motor number
std_msgs/Header header  # ROS timestamp when debug data was computed


# Force sensor calibration chain
float64[] adc_voltage       # [V]      ADC ticks -> voltage (centered: 0V = no force)
float64[] force             # [N]      voltage -> force (before clipping)
float64[] force_clipped     # [N]      after f_max clipping
float64[] torque_raw        # [Nm]     force_clipped * lever_arm -> input to controller

# Admittance integration (rad)
float64[] vdot              # [rad/s^2]  angular acceleration
float64[] velocity_raw      # [rad/s]    after Euler integration (before safety)
float64[] velocity_output   # [rad/s]    after jerk limit + vel clamp (published value)
float64[] displacement      # [rad]      integrated displacement (for K_v > 0)

# Same values in degrees (for readability)
float64[] vdot_deg          # [deg/s^2]
float64[] velocity_raw_deg  # [deg/s]
float64[] velocity_output_deg # [deg/s]
float64[] displacement_deg  # [deg]
