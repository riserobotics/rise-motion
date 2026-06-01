#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rise_motion_messages.msg import MotorPositions
from rise_motion_messages.srv import EnableEthercatSrv, SDOReadSrv
from sdo_serializer import deserialize, serialize


class TestNode(Node):
    def __init__(self, increment: int):
        super().__init__("python_sdo_test_node")
        self.increment = increment
        self.valid_positions = False
        self.motor_pos: list[int] = []

        self.get_logger().info("Starting TestNode")

        self.input_sub = self.create_subscription(
            MotorPositions,
            "motor_feedback",
            self._feedback_cb,
            10,
        )

        self.output_pub = self.create_publisher(MotorPositions, "motor_commands", 10)

        self.publish_timer = self.create_timer(0.001, self._publish_cb)  # 1 ms

        self.enable_client = self.create_client(EnableEthercatSrv, "enable_ethercat")

    def __del__(self):
        self.get_logger().info("Bye :)")

    # ---------------------------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------------------------

    def _feedback_cb(self, msg: MotorPositions):
        if not self.valid_positions:
            self.get_logger().info("Got feedback")
            self.valid_positions = True
        self.motor_pos = list(msg.positions)

    def _publish_cb(self):
        if not self.valid_positions:
            return
        response = MotorPositions()
        response.positions = [p + self.increment for p in self.motor_pos]
        self.output_pub.publish(response)

    # ---------------------------------------------------------------------------
    # Service calls
    # ---------------------------------------------------------------------------

    def request_enable_ethercat(self) -> bool:
        """Returns True on success (mirrors the C++ int == 0 success check)."""
        self.get_logger().info("Incrementing motor position with %d", self.increment)
        self.get_logger().info("Requesting Enable Ethercat")

        while not self.enable_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for enable_ethercat service.")
                return False
            self.get_logger().info("Waiting for enable_ethercat service...")

        request = EnableEthercatSrv.Request()
        request.enable = True

        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("enable_ethercat service call failed.")
            return False

        self.get_logger().info("Done requesting")
        return bool(future.result().status_enable)

    def sdo_read(
        self,
        device_id: int,
        index: int,
        subindex: int,
        value_type: int = 0,
        *,
        type_name: str | None = None,
        base_data_type: str | None = None,
    ):
        """
        Call the sdo_read service and deserialize the result.

        Returns the deserialized value, or None on error.
        """
        client = self.create_client(SDOReadSrv, "sdo_read")

        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for sdo_read service.")
                return None
            self.get_logger().info("Waiting for sdo_read service...")

        request = SDOReadSrv.Request()
        request.device_id = device_id
        request.index = index
        request.subindex = subindex
        request.value_type = value_type

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("sdo_read service call failed.")
            return None

        raw: list[int] = list(future.result().value)
        result = deserialize(raw, name=type_name, base_data_type=base_data_type)

        if result == -1:
            self.get_logger().error("sdo deserialization failed for index 0x%04X sub %d", index, subindex)
            return None

        return result


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    increment = int(sys.argv[1]) if len(sys.argv) >= 2 else 10

    rclpy.init()
    node = TestNode(increment)

    # Keep calling until EtherCAT is enabled
    while not node.request_enable_ethercat():
        pass

    for p in [
        [1,0x1008,0,"VISIBLE_STRING"], [1,0x1000,0,"UDINT"], 
        [1,0x1005,0,"DINT"], [1,0x1018,1,"UDINT"]]:
        # Read device name (object 0x1008, sub 0) as a VISIBLE_STRING
        result = node.sdo_read(
            device_id=p[0],
            index=p[1],
            subindex=p[2],
            type_name=p[3],
        )

        if result is -1:
            rclpy.get_logger("rclcpp").error(f"SDO read {p[1]} {p[3]} failed")
        else:
            node.get_logger().info(f"sdo read value {p[1]} {p[3]}: %s", str(result))

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()