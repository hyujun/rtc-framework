"""TCP (Tool Center Point) visualization for RViz.

Subscribes to GuiPosition messages and generates:
  - Sphere marker at the current TCP position
  - Axes (3 arrows) showing TCP orientation (RPY)
  - TF broadcast for virtual_tcp frame (optional)

Follows the same pattern as SensorVisualizer.
"""

import math

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, TransformStamped
from std_msgs.msg import ColorRGBA


class TcpVisualizer:
    """Converts GuiPosition task_positions [x,y,z,r,p,y] to RViz markers.

    Markers:
      - Sphere at TCP position (semi-transparent)
      - RGB axes arrows (X=red, Y=green, Z=blue) showing orientation
      - Optional: goal sphere (target position, wireframe style)
    """

    def __init__(self, config: dict):
        self.frame_id = config.get('frame_id', 'base')
        self.sphere_radius = config.get('sphere_radius', 0.012)
        self.sphere_color = config.get('sphere_color', [0.2, 0.6, 1.0, 0.7])
        self.axes_length = config.get('axes_length', 0.05)
        self.axes_shaft = config.get('axes_shaft', 0.004)
        self.show_goal = config.get('show_goal', True)
        self.goal_sphere_radius = config.get('goal_sphere_radius', 0.010)
        self.goal_color = config.get('goal_color', [1.0, 0.4, 0.1, 0.5])

    def create_markers(self, task_positions, stamp,
                       goal_positions=None) -> MarkerArray:
        """Create MarkerArray from task_positions [x, y, z, roll, pitch, yaw].

        Args:
            task_positions: list/array of 6 doubles [x, y, z, r, p, y]
            stamp: ROS2 Time message for header stamp
            goal_positions: optional list/array of 6 doubles for goal display
        Returns:
            MarkerArray with TCP visualization markers
        """
        markers = MarkerArray()

        if task_positions is None or len(task_positions) < 3:
            return markers

        x, y, z = float(task_positions[0]), float(task_positions[1]), \
            float(task_positions[2])
        roll = float(task_positions[3]) if len(task_positions) > 3 else 0.0
        pitch = float(task_positions[4]) if len(task_positions) > 4 else 0.0
        yaw = float(task_positions[5]) if len(task_positions) > 5 else 0.0

        # Skip if position is all zeros (no data yet)
        if abs(x) < 1e-9 and abs(y) < 1e-9 and abs(z) < 1e-9:
            return markers

        marker_id = 0

        # TCP sphere
        markers.markers.append(
            self._make_sphere(x, y, z, marker_id, 'tcp_sphere',
                              self.sphere_radius, self.sphere_color, stamp))
        marker_id += 1

        # TCP orientation axes (X=red, Y=green, Z=blue)
        rot = self._rpy_to_matrix(roll, pitch, yaw)
        axis_colors = [
            [1.0, 0.0, 0.0, 0.9],  # X = red
            [0.0, 1.0, 0.0, 0.9],  # Y = green
            [0.0, 0.0, 1.0, 0.9],  # Z = blue
        ]
        for axis_idx in range(3):
            dx = rot[0][axis_idx] * self.axes_length
            dy = rot[1][axis_idx] * self.axes_length
            dz = rot[2][axis_idx] * self.axes_length
            markers.markers.append(
                self._make_axis_arrow(x, y, z, dx, dy, dz,
                                      marker_id, f'tcp_axis_{axis_idx}',
                                      axis_colors[axis_idx], stamp))
            marker_id += 1

        # Goal sphere (optional)
        if self.show_goal and goal_positions is not None \
                and len(goal_positions) >= 3:
            gx = float(goal_positions[0])
            gy = float(goal_positions[1])
            gz = float(goal_positions[2])
            if abs(gx) > 1e-9 or abs(gy) > 1e-9 or abs(gz) > 1e-9:
                markers.markers.append(
                    self._make_sphere(gx, gy, gz, marker_id, 'tcp_goal',
                                      self.goal_sphere_radius,
                                      self.goal_color, stamp))
                marker_id += 1

        return markers

    def create_tf(self, task_positions, stamp,
                  child_frame: str) -> TransformStamped | None:
        """Create TF TransformStamped from task_positions.

        Args:
            task_positions: [x, y, z, roll, pitch, yaw]
            stamp: ROS2 Time message
            child_frame: child frame name (e.g. 'virtual_tcp')
        Returns:
            TransformStamped or None if data insufficient
        """
        if task_positions is None or len(task_positions) < 3:
            return None

        x = float(task_positions[0])
        y = float(task_positions[1])
        z = float(task_positions[2])

        if abs(x) < 1e-9 and abs(y) < 1e-9 and abs(z) < 1e-9:
            return None

        roll = float(task_positions[3]) if len(task_positions) > 3 else 0.0
        pitch = float(task_positions[4]) if len(task_positions) > 4 else 0.0
        yaw = float(task_positions[5]) if len(task_positions) > 5 else 0.0

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.frame_id
        tf.child_frame_id = child_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z

        # RPY → quaternion
        qx, qy, qz, qw = self._rpy_to_quaternion(roll, pitch, yaw)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        return tf

    # ── Private helpers ──────────────────────────────────────────────────

    def _make_sphere(self, x, y, z, marker_id, ns, radius, color, stamp):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        d = radius * 2.0
        marker.scale = Vector3(x=d, y=d, z=d)
        marker.color = ColorRGBA(
            r=float(color[0]), g=float(color[1]),
            b=float(color[2]), a=float(color[3]))
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 200_000_000  # 200ms
        return marker

    def _make_axis_arrow(self, ox, oy, oz, dx, dy, dz,
                         marker_id, ns, color, stamp):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        start = Point(x=ox, y=oy, z=oz)
        end = Point(x=ox + dx, y=oy + dy, z=oz + dz)
        marker.points = [start, end]

        s = self.axes_shaft
        marker.scale = Vector3(x=s, y=s * 2.0, z=0.0)
        marker.color = ColorRGBA(
            r=float(color[0]), g=float(color[1]),
            b=float(color[2]), a=float(color[3]))
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 200_000_000
        return marker

    @staticmethod
    def _rpy_to_matrix(roll, pitch, yaw):
        """RPY (XYZ intrinsic) → 3x3 rotation matrix (row-major lists)."""
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return [
            [cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr],
            [sy * cp,  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr],
            [   -sp,              cp * sr,              cp * cr],
        ]

    @staticmethod
    def _rpy_to_quaternion(roll, pitch, yaw):
        """RPY → quaternion (x, y, z, w)."""
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return qx, qy, qz, qw
