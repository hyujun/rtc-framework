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
from visualization_msgs.msg import MarkerArray

from rtc_msgs.msg import HandSensorState

from rtc_digital_twin.urdf_parser import UrdfParser, JointClassification


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

        # ── URDF parsing and joint classification ─────────────────────────
        self.declare_parameter('auto_compute_mimic', True)
        self._auto_compute_mimic = self.get_parameter('auto_compute_mimic').value

        self._parser: UrdfParser | None = None
        self._joint_classification: JointClassification | None = None
        self._required_joints: set[str] = set()
        self._validation_done = False
        if robot_description:
            try:
                self._parser = UrdfParser.from_xml(robot_description)
                self._joint_classification = self._parser.classification
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
                self.get_logger().error(f'Failed to parse URDF for classification: {e}')

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
            self.declare_parameter(f'{prefix}.joint_names', [''])

            topic = self.get_parameter(f'{prefix}.topic').value
            try:
                raw = self.get_parameter(f'{prefix}.joint_names').value
                joint_names = [n for n in (raw or []) if n]
            except rclpy.exceptions.ParameterUninitializedException:
                joint_names = []

            if not topic:
                self.get_logger().warn(f'source_{i}.topic is empty, skipping')
                continue

            self.get_logger().debug(
                f'source_{i}: topic={topic}, joint_names={joint_names or "(dynamic)"}')

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
        self._fingertip_data = []
        try:
            self.declare_parameter('sensor_viz.sensor_topic', '')
            sensor_topic = self.get_parameter('sensor_viz.sensor_topic').value
            if sensor_topic:
                self.declare_parameter('sensor_viz.marker_topic',
                                       '/digital_twin/fingertip_markers')
                self.declare_parameter('sensor_viz.fingertip_names', [''])
                self.declare_parameter('sensor_viz.barometer_min', 0.0)
                self.declare_parameter('sensor_viz.barometer_max', 1000.0)
                self.declare_parameter('sensor_viz.barometer_arrow_max_length', 0.015)
                self.declare_parameter('sensor_viz.barometer_arrow_scale', 0.0008)
                self.declare_parameter('sensor_viz.tof_enabled', False)
                self.declare_parameter('sensor_viz.tof_max_distance', 0.2)
                self.declare_parameter('sensor_viz.tof_arrow_scale', 0.003)
                self.declare_parameter('sensor_viz.force_arrow_scale', 0.01)
                self.declare_parameter('sensor_viz.force_arrow_shaft', 0.003)
                self.declare_parameter('sensor_viz.displacement_arrow_scale', 0.05)
                self.declare_parameter('sensor_viz.displacement_arrow_shaft', 0.002)
                self.declare_parameter('sensor_viz.contact_sphere_radius', 0.005)

                marker_topic = self.get_parameter('sensor_viz.marker_topic').value
                try:
                    raw_tips = self.get_parameter('sensor_viz.fingertip_names').value
                    fingertip_names = [n for n in (raw_tips or []) if n]
                except rclpy.exceptions.ParameterUninitializedException:
                    fingertip_names = []

                sensor_viz_config = {
                    'barometer_min': self.get_parameter(
                        'sensor_viz.barometer_min').value,
                    'barometer_max': self.get_parameter(
                        'sensor_viz.barometer_max').value,
                    'barometer_arrow_max_length': self.get_parameter(
                        'sensor_viz.barometer_arrow_max_length').value,
                    'barometer_arrow_scale': self.get_parameter(
                        'sensor_viz.barometer_arrow_scale').value,
                    'tof_enabled': self.get_parameter(
                        'sensor_viz.tof_enabled').value,
                    'tof_max_distance': self.get_parameter(
                        'sensor_viz.tof_max_distance').value,
                    'tof_arrow_scale': self.get_parameter(
                        'sensor_viz.tof_arrow_scale').value,
                    'force_arrow_scale': self.get_parameter(
                        'sensor_viz.force_arrow_scale').value,
                    'force_arrow_shaft': self.get_parameter(
                        'sensor_viz.force_arrow_shaft').value,
                    'displacement_arrow_scale': self.get_parameter(
                        'sensor_viz.displacement_arrow_scale').value,
                    'displacement_arrow_shaft': self.get_parameter(
                        'sensor_viz.displacement_arrow_shaft').value,
                    'contact_sphere_radius': self.get_parameter(
                        'sensor_viz.contact_sphere_radius').value,
                }

                from rtc_digital_twin.sensor_visualizer import SensorVisualizer
                self._sensor_viz = SensorVisualizer(
                    fingertip_names, sensor_viz_config)

                self._fingertip_data = [None] * len(fingertip_names)

                self.create_subscription(
                    HandSensorState, sensor_topic, self._sensor_cb, qos)
                self._sensor_pub = self.create_publisher(
                    MarkerArray, marker_topic, 10)
                self._sensor_viz_active = True

                self.get_logger().info(
                    f'Sensor visualization enabled: {sensor_topic} -> {marker_topic} '
                    f'({len(fingertip_names)} fingertips)')
        except Exception as e:
            if sensor_topic:
                self.get_logger().error(
                    f'Failed to initialize sensor visualization: {e}')

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
                    self.get_logger().debug(
                        f'Source {source_idx} ({cache.topic}): first message, '
                        f'{len(msg.name)} joints')
                    cache.received_names = list(msg.name)
                    cache.name_to_idx = {
                        name: i for i, name in enumerate(msg.name)}
                    cache.positions = [0.0] * len(msg.name)
                    cache.velocities = [0.0] * len(msg.name)

                # Update with latest data — use name-based pairing when
                # both name and position arrays are present and same length
                if msg.name and len(msg.name) == len(msg.position):
                    cache.received_names = list(msg.name)
                    cache.positions = list(msg.position)
                    if msg.velocity and len(msg.velocity) == len(msg.name):
                        cache.velocities = list(msg.velocity)
                    else:
                        cache.velocities = [0.0] * len(msg.name)
                elif msg.position:
                    # Fallback: position-only update (no names change)
                    n = min(len(msg.position), len(cache.positions))
                    for i in range(n):
                        cache.positions[i] = msg.position[i]
            else:
                # Static mode: reorder by name→idx mapping
                for i, name in enumerate(msg.name):
                    idx = cache.name_to_idx.get(name)
                    if idx is not None:
                        if i < len(msg.position):
                            cache.positions[idx] = msg.position[i]
                        if msg.velocity and i < len(msg.velocity):
                            cache.velocities[idx] = msg.velocity[i]

            if not cache.data_received:
                self.get_logger().info(
                    f'Source {source_idx} ({cache.topic}): receiving data')
            cache.data_received = True

        return callback

    def _sensor_cb(self, msg: HandSensorState):
        """Cache latest fingertip sensor data from HandSensorState."""
        n_tips = min(len(msg.fingertips), len(self._fingertip_data))
        for i in range(n_tips):
            self._fingertip_data[i] = msg.fingertips[i]

    def _publish_display(self):
        """Timer callback — publish combined JointState + optional markers."""
        now = self.get_clock().now().to_msg()

        # If no data received yet, publish all URDF joints at position 0
        if not any(s.data_received for s in self._sources):
            if self._joint_classification:
                js = JointState()
                js.header.stamp = now
                all_joints = sorted(self._joint_classification.active_names
                                    | self._joint_classification.passive_names)
                js.name = all_joints
                js.position = [0.0] * len(all_joints)
                js.velocity = [0.0] * len(all_joints)
                self._joint_pub.publish(js)
            return

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
                and self._parser
                and self._joint_classification
                and self._joint_classification.passive_mimic):
            positions_map = dict(zip(js.name, js.position))
            mimic_positions = self._parser.compute_mimic_positions(
                positions_map)
            for name, pos in mimic_positions.items():
                if name not in positions_map:
                    js.name.append(name)
                    js.position.append(pos)
                    js.velocity.append(0.0)

        self._joint_pub.publish(js)

        # Sensor visualization
        if self._sensor_viz_active and self._sensor_viz:
            markers = self._sensor_viz.create_markers(
                self._fingertip_data, now)
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

        covered, missing = self._parser.validate_joints(received)

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

        _, missing = self._parser.validate_joints(received)
        if missing:
            self.get_logger().warn(
                f'Missing {len(missing)} required joints: {sorted(missing)}')


def main(args=None):
    rclpy.init(args=args)
    node = DigitalTwinNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down (KeyboardInterrupt)')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
