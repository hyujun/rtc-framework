"""Generalized Digital Twin Node.

Subscribes to configurable JointState topics (RELIABLE, depth 10),
merges them into a single combined JointState for robot_state_publisher,
and validates that all required URDF joints are covered.

Optionally activates fingertip sensor visualization (MarkerArray) when
sensor_viz parameters are present in the config.
"""

from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import MarkerArray

from rtc_digital_twin.urdf_validator import (
    classify_joints,
    compute_mimic_positions,
    JointClassification,
    validate_joints,
)


@dataclass
class JointStateCache:
    """Per-source joint state cache."""
    topic: str
    joint_names: list[str]
    name_to_idx: dict[str, int] = field(default_factory=dict)
    positions: list[float] = field(default_factory=list)
    velocities: list[float] = field(default_factory=list)
    data_received: bool = False
    dynamic: bool = False  # True when joint_names is empty (accept all)
    # For dynamic mode: track received joint names in order
    received_names: list[str] = field(default_factory=list)

    def __post_init__(self):
        if self.joint_names:
            n = len(self.joint_names)
            self.name_to_idx = {name: i for i, name in enumerate(self.joint_names)}
            self.positions = [0.0] * n
            self.velocities = [0.0] * n
            self.dynamic = False
        else:
            self.dynamic = True


class DigitalTwinNode(Node):
    def __init__(self):
        super().__init__('digital_twin_node')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('display_rate', 60.0)
        self.declare_parameter('output_topic', '/digital_twin/joint_states')
        self.declare_parameter('num_sources', 1)
        self.declare_parameter('robot_description', '')

        display_rate = self.get_parameter('display_rate').value
        output_topic = self.get_parameter('output_topic').value
        num_sources = self.get_parameter('num_sources').value
        robot_description = self.get_parameter('robot_description').value

        # ── URDF joint classification ─────────────────────────────────────
        self.declare_parameter('auto_compute_mimic', True)
        self._auto_compute_mimic = self.get_parameter('auto_compute_mimic').value

        self._joint_classification: JointClassification | None = None
        self._required_joints: set[str] = set()
        self._validation_done = False
        if robot_description:
            try:
                self._joint_classification = classify_joints(robot_description)
                self._required_joints = self._joint_classification.active_names
                logger = self.get_logger()
                logger.info(
                    f'Active joints ({len(self._joint_classification.active)}): '
                    f'{sorted(self._joint_classification.active.keys())}')
                if self._joint_classification.passive_mimic:
                    logger.info(
                        f'Mimic joints ({len(self._joint_classification.passive_mimic)}): '
                        f'{sorted(self._joint_classification.passive_mimic.keys())}')
                if self._joint_classification.passive_closed_chain:
                    logger.info(
                        f'Closed-chain joints '
                        f'({len(self._joint_classification.passive_closed_chain)}): '
                        f'{sorted(self._joint_classification.passive_closed_chain.keys())}')
            except Exception as e:
                self.get_logger().warn(f'Failed to parse URDF for classification: {e}')

        # ── QoS — RELIABLE, depth 10 ────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Joint state sources ──────────────────────────────────────────
        self._sources: list[JointStateCache] = []
        for i in range(num_sources):
            prefix = f'source_{i}'
            self.declare_parameter(f'{prefix}.topic', '')
            self.declare_parameter(
                f'{prefix}.joint_names',
                rclpy.Parameter.Type.STRING_ARRAY,
            )

            topic = self.get_parameter(f'{prefix}.topic').value
            raw = self.get_parameter(f'{prefix}.joint_names').value
            joint_names = list(raw) if raw else []

            if not topic:
                self.get_logger().warn(f'source_{i}.topic is empty, skipping')
                continue

            cache = JointStateCache(topic=topic, joint_names=joint_names)
            self._sources.append(cache)

            idx = len(self._sources) - 1
            self.create_subscription(
                JointState, topic, self._make_source_callback(idx), qos)
            mode = 'dynamic' if cache.dynamic else f'{len(joint_names)} joints'
            self.get_logger().info(f'Source {i}: {topic} ({mode})')

        # ── Sensor visualization (optional) ──────────────────────────────
        self._sensor_viz_active = False
        self._sensor_viz = None
        self._sensor_pub = None
        try:
            self.declare_parameter('sensor_viz.sensor_topic', '')
            sensor_topic = self.get_parameter('sensor_viz.sensor_topic').value
            if sensor_topic:
                self.declare_parameter('sensor_viz.marker_topic',
                                       '/digital_twin/fingertip_markers')
                self.declare_parameter(
                    'sensor_viz.fingertip_names',
                    rclpy.Parameter.Type.STRING_ARRAY,
                )
                self.declare_parameter('sensor_viz.barometer_min', 0.0)
                self.declare_parameter('sensor_viz.barometer_max', 1000.0)
                self.declare_parameter('sensor_viz.barometer_sphere_min', 0.002)
                self.declare_parameter('sensor_viz.barometer_sphere_max', 0.008)
                self.declare_parameter('sensor_viz.tof_max_distance', 0.2)
                self.declare_parameter('sensor_viz.tof_arrow_scale', 0.003)

                marker_topic = self.get_parameter('sensor_viz.marker_topic').value
                raw_tips = self.get_parameter('sensor_viz.fingertip_names').value
                fingertip_names = list(raw_tips) if raw_tips else []

                sensor_viz_config = {
                    'barometer_min': self.get_parameter(
                        'sensor_viz.barometer_min').value,
                    'barometer_max': self.get_parameter(
                        'sensor_viz.barometer_max').value,
                    'barometer_sphere_min': self.get_parameter(
                        'sensor_viz.barometer_sphere_min').value,
                    'barometer_sphere_max': self.get_parameter(
                        'sensor_viz.barometer_sphere_max').value,
                    'tof_max_distance': self.get_parameter(
                        'sensor_viz.tof_max_distance').value,
                    'tof_arrow_scale': self.get_parameter(
                        'sensor_viz.tof_arrow_scale').value,
                }

                from rtc_digital_twin.sensor_visualizer import SensorVisualizer
                self._sensor_viz = SensorVisualizer(
                    fingertip_names, sensor_viz_config)

                n_tips = len(fingertip_names)
                self._fingertip_sensors = [[0.0] * 11 for _ in range(n_tips)]

                self.create_subscription(
                    Float64MultiArray, sensor_topic, self._sensor_cb, qos)
                self._sensor_pub = self.create_publisher(
                    MarkerArray, marker_topic, 10)
                self._sensor_viz_active = True

                self.get_logger().info(
                    f'Sensor visualization enabled: {sensor_topic} -> {marker_topic}')
        except Exception:
            pass

        # ── Publisher ────────────────────────────────────────────────────
        self._joint_pub = self.create_publisher(JointState, output_topic, 10)

        # ── Display timer ────────────────────────────────────────────────
        timer_period = 1.0 / display_rate
        self.create_timer(timer_period, self._publish_display)

        # ── Validation timer (first at 3s, then every 10s) ──────────────
        if self._required_joints:
            self._validation_timer = self.create_timer(
                3.0, self._validate_joints)

        self.get_logger().info(
            f'Digital Twin node started: {len(self._sources)} source(s), '
            f'display_rate={display_rate}Hz, output={output_topic}')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _make_source_callback(self, source_idx: int):
        """Create a closure that updates the source cache at source_idx."""
        def callback(msg: JointState):
            cache = self._sources[source_idx]

            if cache.dynamic:
                # Dynamic mode: accept all joints, grow cache on first message
                if not cache.data_received and msg.name:
                    cache.received_names = list(msg.name)
                    cache.name_to_idx = {
                        name: i for i, name in enumerate(msg.name)}
                    cache.positions = [0.0] * len(msg.name)
                    cache.velocities = [0.0] * len(msg.name)

                # Update with latest data (pass-through)
                if msg.name:
                    cache.received_names = list(msg.name)
                cache.positions = list(msg.position)
                if msg.velocity:
                    cache.velocities = list(msg.velocity)
            else:
                # Static mode: reorder by name→idx mapping
                for i, name in enumerate(msg.name):
                    idx = cache.name_to_idx.get(name)
                    if idx is not None:
                        if i < len(msg.position):
                            cache.positions[idx] = msg.position[i]
                        if msg.velocity and i < len(msg.velocity):
                            cache.velocities[idx] = msg.velocity[i]

            cache.data_received = True

        return callback

    def _sensor_cb(self, msg: Float64MultiArray):
        """Cache latest fingertip sensor data."""
        data = msg.data
        n_tips = len(self._fingertip_sensors)
        expected = n_tips * 11
        if len(data) < expected:
            return
        for i in range(n_tips):
            offset = i * 11
            self._fingertip_sensors[i] = list(data[offset:offset + 11])

    def _publish_display(self):
        """Timer callback — publish combined JointState + optional markers."""
        # Check if any source has data
        if not any(s.data_received for s in self._sources):
            return

        now = self.get_clock().now().to_msg()

        # Merge all sources into a single JointState
        js = JointState()
        js.header.stamp = now

        for cache in self._sources:
            if not cache.data_received:
                continue
            if cache.dynamic:
                js.name.extend(cache.received_names)
            else:
                js.name.extend(cache.joint_names)
            js.position.extend(cache.positions)
            js.velocity.extend(cache.velocities)

        # Auto-compute mimic joint positions
        if (self._auto_compute_mimic
                and self._joint_classification
                and self._joint_classification.passive_mimic):
            positions_map = dict(zip(js.name, js.position))
            mimic_positions = compute_mimic_positions(
                self._joint_classification, positions_map)
            for name, pos in mimic_positions.items():
                if name not in positions_map:
                    js.name.append(name)
                    js.position.append(pos)
                    js.velocity.append(0.0)

        self._joint_pub.publish(js)

        # Sensor visualization
        if self._sensor_viz_active and self._sensor_viz:
            markers = self._sensor_viz.create_markers(
                self._fingertip_sensors, now)
            self._sensor_pub.publish(markers)

    def _validate_joints(self):
        """Periodic validation: check received joints against URDF."""
        received = set()
        for cache in self._sources:
            if not cache.data_received:
                continue
            if cache.dynamic:
                received.update(cache.received_names)
            else:
                received.update(cache.joint_names)

        if not received:
            self.get_logger().warn('No joint data received yet')
            return

        covered, missing = validate_joints(self._required_joints, received)

        if missing:
            self.get_logger().warn(
                f'Missing {len(missing)} required joints: {sorted(missing)}')
        elif not self._validation_done:
            self.get_logger().info(
                f'All {len(covered)} required joints covered')
            self._validation_done = True

        # After first validation, switch to 10s interval
        if hasattr(self, '_validation_timer'):
            self._validation_timer.cancel()
            self._validation_timer = self.create_timer(
                10.0, self._validate_joints_periodic)

    def _validate_joints_periodic(self):
        """Periodic re-validation (10s interval)."""
        received = set()
        for cache in self._sources:
            if not cache.data_received:
                continue
            if cache.dynamic:
                received.update(cache.received_names)
            else:
                received.update(cache.joint_names)

        if not received:
            return

        _, missing = validate_joints(self._required_joints, received)
        if missing:
            self.get_logger().warn(
                f'Missing {len(missing)} required joints: {sorted(missing)}')


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
