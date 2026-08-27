# Per-motor debug data for admittance controller
# Arrays are indexed by motor number
std_msgs/Header header  # ROS timestamp when debug data was computed


# Force sensor calibration chain
float64[] adc_voltage_raw   # [V]      ADC ticks -> voltage (uncalibrated, 0..5V range)
float64[] adc_voltage       # [V]      ADC ticks -> voltage (centered: 0V = no force)
float64[] force             # [N]      voltage -> force (before clipping)
float64[] force_clipped     # [N]      after f_max clipping
float64[] force_filtered    # [N]      after EMA low-pass filter (input to controller)
float64[] torque_raw        # [Nm]     force_clipped * lever_arm -> input to controller

# Absolute motor position
float64[] pos_rad           # [rad]      absolute encoder position

# Position relative to the user-settable tare point
float64[] pos_rel_rad       # [rad]      pos_rad - tare offset
float64[] pos_rel_deg       # [deg]      same, in degrees

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

# Measured joint velocity and observed acceleration
float64[] velocity_fb_joint # [rad/s]    feedback velocity converted to joint speed
float64[] accel_raw         # [rad/s^2]  differentiated feedback velocity
float64[] accel_filtered    # [rad/s^2]  filtered feedback acceleration

# Compensation and measured torque at joint level
float64[] tau_gravity_nm    # [Nm]
float64[] tau_friction_nm   # [Nm]
float64[] torque_fb         # [Nm]      measured motor torque converted to joint torque
