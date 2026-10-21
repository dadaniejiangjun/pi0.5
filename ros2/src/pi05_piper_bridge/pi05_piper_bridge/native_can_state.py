"""Publish native SocketCAN feedback using the existing project decoder."""
import json
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from pi05_piper_interfaces.msg import StateSnapshot

from laptop.robot_client.piper_session import PiperSession
from laptop.ros_gateway.windows_read_only_gateway import piper_state_to_wire
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.socketcan_receive import SocketCanReceiveTransport


class NativeCanState(Node):
    def __init__(self, bridge_factory=None):
        super().__init__("pi05_native_can_state")
        self.declare_parameter("interface", "can0")
        self.declare_parameter("log_path", "/tmp/pi05_native_can.jsonl")
        self.declare_parameter("session_id", "")
        self.declare_parameter("allow_virtual_test", False)
        self.declare_parameter("log_rx_frames", False)
        self.declare_parameter("state_publish_hz", 10.0)
        state_hz = float(self.get_parameter('state_publish_hz').value)
        if not 1 <= state_hz <= 50:
            raise ValueError('state_publish_hz must be within 1..50')
        interface = str(self.get_parameter("interface").value)
        self.virtual = interface.startswith("vcan")
        if self.virtual and not self.get_parameter("allow_virtual_test").value:
            raise PermissionError("vcan requires explicit allow_virtual_test; not real hardware evidence")
        self.session_id = str(self.get_parameter("session_id").value) or uuid.uuid4().hex
        if bridge_factory is None:
            transport = SocketCanReceiveTransport(interface)
            bridge = SafetyBridge(mode=RuntimeMode.POLICY_REJECT_ONLY,
                                  transport=transport, limits=UNVERIFIED_ANALYSIS_LIMITS)
        else:
            bridge = bridge_factory(self, interface)
        path = Path(str(self.get_parameter("log_path").value))
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.exists():
            raise FileExistsError(path)
        self.session = PiperSession(path, bridge=bridge,
            log_rx_frames=bool(self.get_parameter("log_rx_frames").value)).connect()
        self.state_pub = self.create_publisher(StateSnapshot, "/piper/state_snapshot", qos_profile_sensor_data)
        self.joint_pub = self.create_publisher(JointState, "/piper/feedback/joint_states", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(DiagnosticArray, "/piper/status", 10)
        self.sequence = 0
        self.frame_count = 0
        self.sample_count = 0
        self.create_timer(0.01, self.poll)
        self.create_timer(1.0/state_hz, self.publish)
        self.get_logger().info(f"native SocketCAN session={self.session_id}; manual_backend={bridge_factory is not None}")

    def poll(self):
        self.frame_count += self.session.poll(0.005)

    def publish(self):
        state = self.session.state()
        if state["complete_fresh"]:
            self.sequence += 1
            wire = piper_state_to_wire(state, self.sequence)
            wire.update(session_id=self.session_id, fake=self.virtual, hardware_connected=not self.virtual)
            msg = StateSnapshot()
            for key in ("session_id", "sequence", "feedback_fresh", "joint_skew_ns",
                        "status_monotonic_ns", "joint_pair_monotonic_ns", "gripper_monotonic_ns",
                        "motor_monotonic_ns", "joint_position_rad", "gripper_raw", "gripper_nominal_m",
                        "ctrl_mode_raw", "arm_status_raw", "error_code", "motor_enabled",
                        "hardware_connected", "fake"):
                setattr(msg, key, self.session_id if key == "session_id" else wire[key])
            msg.source_monotonic_ns = wire["host_monotonic_ns"]
            msg.source_local_monotonic_ns = msg.source_monotonic_ns
            msg.source_clock_id = "linux_perf_counter_ns"
            msg.source_local_clock_id = "wsl_perf_counter_ns"
            msg.source_clock_mapping_uncertainty_ns = 0
            stamp = self.get_clock().now().nanoseconds - (time.perf_counter_ns() - msg.source_monotonic_ns)
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            msg.header.frame_id = msg.source_clock_id
            self.state_pub.publish(msg)
            joint = JointState(header=msg.header)
            joint.name = [f"joint{i}" for i in range(1, 7)] + ["gripper"]
            joint.position = list(msg.joint_position_rad) + [msg.gripper_nominal_m]
            self.joint_pub.publish(joint)
            self.session.record_state_snapshot(wire)
            self.on_state(wire)
            self.sample_count += 1
        status = DiagnosticStatus(name="pi05/native_can", hardware_id=self.session_id)
        status.level = DiagnosticStatus.OK if state["complete_fresh"] else DiagnosticStatus.WARN
        status.message = "fresh feedback" if state["complete_fresh"] else "missing or stale CAN feedback"
        status.values = [KeyValue(key=k, value=json.dumps(v)) for k, v in {
            "session_id": self.session_id, "rx_frames": self.frame_count,
            "published_samples": self.sample_count, "missing_channels": state["missing_channels"],
            "stale_channels": state["stale_channels"], "can_tx_calls": getattr(getattr(self, 'manual_adapter', None), 'tx_calls', 0),
            "hardware_commands_enabled": getattr(self, 'manual_enabled', False)}.items()]
        report = DiagnosticArray(status=[status])
        report.header.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(report)

    def close(self):
        self.session.close()

    def on_state(self, wire):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NativeCanState()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
