"""Print changed ROS Joy states for button and axis identification."""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy


class InputEcho(Node):
    def __init__(self):
        super().__init__("pi05_gamepad_input_echo")
        self.last_state = None
        self.create_subscription(Joy, "/joy", self.on_joy, qos_profile_sensor_data)
        self.get_logger().info("read-only /joy preview; no robot topics are read or published")

    def on_joy(self, message: Joy) -> None:
        state = (tuple(round(float(value), 4) for value in message.axes),
                 tuple(int(value) for value in message.buttons))
        if state == self.last_state:
            return
        self.last_state = state
        self.get_logger().info(json.dumps({
            "schema": "piper_gamepad_input_preview_v1",
            "host_monotonic_ns": time.perf_counter_ns(),
            "axes": state[0],
            "buttons": state[1],
        }, separators=(",", ":")))


def main(args=None):
    rclpy.init(args=args)
    node = InputEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
