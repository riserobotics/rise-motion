# Per-motor debug data for admittance controller
# Arrays are indexed by motor number
std_msgs/Header header  # ROS timestamp when debug data was computed


# Force sensor calibration chain
float64[] adc_voltage_raw   # [V]      ADC ticks -> voltage (uncalibrated, 0..5V range)
float64[] adc_voltage       # [V]      ADC ticks -> voltage (centered: 0V = no force)
float64[] force             # [N]      voltage -> force (before clipping)
float64[] force_clipped     # [N]      after f_max clipping
float64[] force_filtered    # [N]      after EMA low-pass filter (input to controller)
float64[] torque_raw        # [Nm]     force_filtered * lever_arm -> input to controller

# Absolute motor position (encoder ticks / enc_per_rad) — same reference as pos_min/max limits
float64[] pos_rad           # [rad]      absolute encoder position

# Position relative to a user-settable zero point (tare), for experiments where the raw
# absolute encoder origin is meaningless (e.g. sinus-reference tracking). Zeroed via the
# ~/zero_position service (captures current pos_rad as the new origin). Purely informational
# — never used for pos_min/max limit checks or any control law, those stay on pos_rad.
float64[] pos_rel_rad        # [rad]      pos_rad - offset from last ~/zero_position call
float64[] pos_rel_deg        # [deg]      same, in degrees

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

# Measured joint velocity (velocity_fb converted from motor RPM to joint rad/s)
float64[] velocity_fb_joint  # [rad/s]  velocity_value [RPM] * 2π/60 / gear_ratio

# Compensation torques at joint level [Nm] — observer only, same formula as comp functions
float64[] tau_gravity_nm    # [Nm]  mgl * cos(pos_rad + angle_offset); 0 if disabled
float64[] tau_friction_nm   # [Nm]  (tau_c * tanh(k*v) + b*v) * scale; 0 if disabled or |v|>max_vel

# Observed joint acceleration (differentiated velocity_fb, no controller feedback)
float64[] accel_raw       # [rad/s^2]  (v_fb[k] - v_fb[k-1]) / dt  — joint-level
float64[] accel_filtered  # [rad/s^2]  EMA low-pass (accel_filter_cutoff_hz)

# Measured motor torque (0x6077), converted to joint [Nm]
# = torque_value [‰ rated] / 1000 * rated_torque_motor [Nm] * gear_ratio
float64[] torque_fb       # [Nm]  at joint
