"""Digital Twin Node — 500Hz topic subscriber + 60Hz RViz publisher.

Subscribes to /joint_states (UR5e) and /hand/joint_states (Custom Hand),
caches the latest state, and publishes combined JointState + MarkerArray
at a configurable display rate (default 60Hz) for robot_state_publisher
and RViz2 visualization.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import MarkerArray

from rtc_digital_twin.sensor_visualizer import SensorVisualizer


class DigitalTwinNode(Node):
    def __init__(self):
        super().__init__('digital_twin_node')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('display_rate', 60.0)
        self.declare_parameter('enable_hand', True)
        self.declare_parameter('robot_joint_states_topic', '/joint_states')
        self.declare_parameter('hand_joint_states_topic', '/hand/joint_states')

        self.declare_parameter('robot_joint_names', [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
        ])
        self.declare_parameter('hand_motor_names', [
            'thumb_cmc_aa', 'thumb_cmc_fe', 'thumb_mcp_fe',
            'index_mcp_aa', 'index_mcp_fe', 'index_dip_fe',
            'middle_mcp_aa', 'middle_mcp_fe', 'middle_dip_fe',
            'ring_mcp_fe',
        ])
        self.declare_parameter('fingertip_names', [
            'thumb', 'index', 'middle', 'ring',
        ])

        # Sensor visualization parameters
        self.declare_parameter('sensor_viz.barometer_min', 0.0)
        self.declare_parameter('sensor_viz.barometer_max', 1000.0)
        self.declare_parameter('sensor_viz.barometer_sphere_min', 0.002)
        self.declare_parameter('sensor_viz.barometer_sphere_max', 0.008)
        self.declare_parameter('sensor_viz.tof_max_distance', 0.2)
        self.declare_parameter('sensor_viz.tof_arrow_scale', 0.003)

        # Read parameters
        self._display_rate = self.get_parameter('display_rate').value
        self._enable_hand = self.get_parameter('enable_hand').value
        robot_topic = self.get_parameter('robot_joint_states_topic').value
        hand_topic = self.get_parameter('hand_joint_states_topic').value
        self._robot_joint_names = list(self.get_parameter('robot_joint_names').value)
        self._hand_motor_names = list(self.get_parameter('hand_motor_names').value)
        fingertip_names = list(self.get_parameter('fingertip_names').value)

        sensor_viz_config = {
            'barometer_min': self.get_parameter('sensor_viz.barometer_min').value,
            'barometer_max': self.get_parameter('sensor_viz.barometer_max').value,
            'barometer_sphere_min': self.get_parameter('sensor_viz.barometer_sphere_min').value,
            'barometer_sphere_max': self.get_parameter('sensor_viz.barometer_sphere_max').value,
            'tof_max_distance': self.get_parameter('sensor_viz.tof_max_distance').value,
            'tof_arrow_scale': self.get_parameter('sensor_viz.tof_arrow_scale').value,
        }

        # ── State cache ─────────────────────────────────────────────────
        n_robot = len(self._robot_joint_names)
        n_hand = len(self._hand_motor_names)
        n_tips = len(fingertip_names)

        self._robot_positions = [0.0] * n_robot
        self._robot_velocities = [0.0] * n_robot
        self._hand_positions = [0.0] * n_hand
        self._hand_velocities = [0.0] * n_hand
        self._fingertip_sensors = [[0.0] * 11 for _ in range(n_tips)]
        self._robot_data_received = False
        self._hand_data_received = False

        # Name → index mapping for robot joints (handles reordering)
        self._robot_name_to_idx = {
            name: i for i, name in enumerate(self._robot_joint_names)
        }

        # ── QoS — best effort, depth 1 (latest-only) ───────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ─────────────────────────────────────────────────
        self.create_subscription(
            JointState, robot_topic, self._robot_cb, qos
        )
        if self._enable_hand:
            self.create_subscription(
                Float64MultiArray, hand_topic, self._hand_cb, qos
            )

        # ── Publishers ──────────────────────────────────────────────────
        self._joint_pub = self.create_publisher(
            JointState, '/digital_twin/joint_states', 10
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, '/digital_twin/fingertip_markers', 10
        )

        # ── Sensor visualizer ───────────────────────────────────────────
        self._sensor_viz = SensorVisualizer(fingertip_names, sensor_viz_config)

        # ── Display timer ───────────────────────────────────────────────
        timer_period = 1.0 / self._display_rate
        self.create_timer(timer_period, self._publish_display)

        self.get_logger().info(
            f'Digital Twin node started: display_rate={self._display_rate}Hz, '
            f'enable_hand={self._enable_hand}'
        )

    # ── Callbacks ────────────────────────────────────────────────────────

    def _robot_cb(self, msg: JointState):
        """Cache latest robot joint state (500Hz input → cache only)."""
        for i, name in enumerate(msg.name):
            idx = self._robot_name_to_idx.get(name)
            if idx is not None:
                if i < len(msg.position):
                    self._robot_positions[idx] = msg.position[i]
                if msg.velocity and i < len(msg.velocity):
                    self._robot_velocities[idx] = msg.velocity[i]
        self._robot_data_received = True

    def _hand_cb(self, msg: Float64MultiArray):
        """Cache latest hand state. Format: [pos×10, vel×10, sensor×44] = 64 doubles."""
        data = msg.data
        if len(data) < 64:
            return
        self._hand_positions = list(data[0:10])
        self._hand_velocities = list(data[10:20])
        for i in range(len(self._fingertip_sensors)):
            offset = 20 + i * 11
            self._fingertip_sensors[i] = list(data[offset:offset + 11])
        self._hand_data_received = True

    # ── 60Hz display publish ─────────────────────────────────────────────

    def _publish_display(self):
        """Timer callback — publish combined JointState + sensor markers."""
        if not self._robot_data_received:
            return

        now = self.get_clock().now().to_msg()

        # 1. Combined JointState → robot_state_publisher
        js = JointState()
        js.header.stamp = now
        js.name = list(self._robot_joint_names)
        js.position = list(self._robot_positions)
        js.velocity = list(self._robot_velocities)

        if self._enable_hand and self._hand_data_received:
            js.name += self._hand_motor_names
            js.position += self._hand_positions
            js.velocity += self._hand_velocities

        self._joint_pub.publish(js)

        # 2. Fingertip sensor MarkerArray
        if self._enable_hand and self._hand_data_received:
            markers = self._sensor_viz.create_markers(
                self._fingertip_sensors, now
            )
            self._marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = DigitalTwinNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
