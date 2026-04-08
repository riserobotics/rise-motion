#!/usr/bin/env python3
"""
Publishes a sine wave on /analog_input_override (std_msgs/UInt16, ADC ticks).

ADC range: 0..65535
  0     = 0V   = max negative force
  32768 = 2.5V = 0N (neutral)
  65535 = 5V   = max positive force

Usage:
  python3 sine_force_publisher.py [amplitude_n] [frequency_hz]

  amplitude_n:   Force amplitude in N (default: 25.0)
  frequency_hz:  Sine frequency in Hz (default: 0.5)

Examples:
  python3 sine_force_publisher.py          # 25N @ 0.5Hz
  python3 sine_force_publisher.py 10 1.0   # 10N @ 1Hz
"""

import sys
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16

# ADC calibration (must match admittance_node parameters)
SENSITIVITY_INV = 10.0  # [N/V] — same default as admittance_node
ADC_MAX = 65535
V_RANGE = 5.0
V_OFFSET = 2.5  # 2.5V = 0N


def force_to_adc(force_n: float) -> int:
    """Convert force [N] to ADC ticks."""
    voltage = force_n / SENSITIVITY_INV + V_OFFSET
    adc = int(voltage / V_RANGE * ADC_MAX)
    return max(0, min(ADC_MAX, adc))


def main():
    amplitude_n = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
    frequency_hz = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5

    rclpy.init()
    node = Node('sine_force_publisher')
    pub = node.create_publisher(UInt16, 'analog_input_override', 10)

    node.get_logger().info(
        f"Publishing sine: amplitude={amplitude_n}N, frequency={frequency_hz}Hz"
    )
    node.get_logger().info(
        f"ADC range: {force_to_adc(-amplitude_n)} .. {force_to_adc(amplitude_n)} "
        f"(neutral=32768)"
    )

    dt = 0.01  # 100Hz publish rate
    t = 0.0
    msg = UInt16()

    while rclpy.ok():
        force = amplitude_n * math.sin(2.0 * math.pi * frequency_hz * t)
        msg.data = force_to_adc(force)
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0)
        time.sleep(dt)
        t += dt

    rclpy.shutdown()


if __name__ == '__main__':
    main()
