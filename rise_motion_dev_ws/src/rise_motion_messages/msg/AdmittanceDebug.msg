# Per-motor debug data for admittance controller
# Arrays are indexed by motor number

# Force sensor calibration chain
float64[] adc_voltage       # [V]  ADC ticks -> voltage
float64[] force             # [N]  voltage -> force
float64[] force_clipped     # [N]  after f_max clipping
float64[] torque_raw        # [Nm] force_clipped * lever_arm

# Admittance integration
float64[] vdot              # [enc-inc/s^2] acceleration
float64[] velocity_raw      # [enc-inc/s]   after Euler integration (before safety)
float64[] velocity_output   # [enc-inc/s]   after jerk limit + vel clamp (published value)
float64[] displacement      # [enc-inc]     integrated displacement (for K_v > 0)
