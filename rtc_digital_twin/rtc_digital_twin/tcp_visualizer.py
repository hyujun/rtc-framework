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
                  parent_frame: str,
                  child_frame: str,
                  T_base_parent=None) -> TransformStamped | None:
        """Create TF TransformStamped for parent_frame → child_frame.

        When T_base_parent is provided, computes the relative transform:
          T_parent_child = inv(T_base_parent) * T_base_child
        Otherwise falls back to broadcasting in base frame directly.

        Args:
            task_positions: [x, y, z, roll, pitch, yaw] in base frame
            stamp: ROS2 Time message
            parent_frame: TF parent frame (e.g. 'tool0')
            child_frame: TF child frame (e.g. 'virtual_tcp')
            T_base_parent: optional (translation, quaternion) tuple for
                           base→parent transform. quaternion = (x, y, z, w)
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

        # T_base_child: base → virtual_tcp
        qx_c, qy_c, qz_c, qw_c = self._rpy_to_quaternion(roll, pitch, yaw)
        R_base_child = self._quat_to_matrix(qx_c, qy_c, qz_c, qw_c)
        t_base_child = [x, y, z]

        if T_base_parent is not None:
            # Compute relative: T_parent_child = inv(T_base_parent) * T_base_child
            (tp, qp) = T_base_parent  # tp=(x,y,z), qp=(x,y,z,w)
            R_base_parent = self._quat_to_matrix(qp[0], qp[1], qp[2], qp[3])
            t_base_parent = [tp[0], tp[1], tp[2]]

            # inv(R_base_parent)
            R_parent_base = self._transpose3(R_base_parent)
            # t_rel = R_parent_base * (t_child - t_parent)
            dx = t_base_child[0] - t_base_parent[0]
            dy = t_base_child[1] - t_base_parent[1]
            dz = t_base_child[2] - t_base_parent[2]
            tx = R_parent_base[0][0]*dx + R_parent_base[0][1]*dy + R_parent_base[0][2]*dz
            ty = R_parent_base[1][0]*dx + R_parent_base[1][1]*dy + R_parent_base[1][2]*dz
            tz = R_parent_base[2][0]*dx + R_parent_base[2][1]*dy + R_parent_base[2][2]*dz
            # R_rel = R_parent_base * R_base_child
            R_rel = self._matmul3(R_parent_base, R_base_child)
            qx_r, qy_r, qz_r, qw_r = self._matrix_to_quat(R_rel)

            out_t = (tx, ty, tz)
            out_q = (qx_r, qy_r, qz_r, qw_r)
        else:
            out_t = (x, y, z)
            out_q = (qx_c, qy_c, qz_c, qw_c)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = parent_frame
        tf.child_frame_id = child_frame
        tf.transform.translation.x = out_t[0]
        tf.transform.translation.y = out_t[1]
        tf.transform.translation.z = out_t[2]
        tf.transform.rotation.x = out_q[0]
        tf.transform.rotation.y = out_q[1]
        tf.transform.rotation.z = out_q[2]
        tf.transform.rotation.w = out_q[3]

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

    @staticmethod
    def _quat_to_matrix(qx, qy, qz, qw):
        """Quaternion (x, y, z, w) → 3x3 rotation matrix."""
        xx, yy, zz = qx*qx, qy*qy, qz*qz
        xy, xz, yz = qx*qy, qx*qz, qy*qz
        wx, wy, wz = qw*qx, qw*qy, qw*qz
        return [
            [1 - 2*(yy+zz),     2*(xy-wz),     2*(xz+wy)],
            [    2*(xy+wz), 1 - 2*(xx+zz),     2*(yz-wx)],
            [    2*(xz-wy),     2*(yz+wx), 1 - 2*(xx+yy)],
        ]

    @staticmethod
    def _transpose3(m):
        """Transpose a 3x3 matrix (list of lists)."""
        return [
            [m[0][0], m[1][0], m[2][0]],
            [m[0][1], m[1][1], m[2][1]],
            [m[0][2], m[1][2], m[2][2]],
        ]

    @staticmethod
    def _matmul3(a, b):
        """Multiply two 3x3 matrices."""
        return [
            [sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)
        ]

    @staticmethod
    def _matrix_to_quat(m):
        """3x3 rotation matrix → quaternion (x, y, z, w)."""
        tr = m[0][0] + m[1][1] + m[2][2]
        if tr > 0:
            s = 2.0 * math.sqrt(tr + 1.0)
            w = 0.25 * s
            x = (m[2][1] - m[1][2]) / s
            y = (m[0][2] - m[2][0]) / s
            z = (m[1][0] - m[0][1]) / s
        elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
            s = 2.0 * math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2])
            w = (m[2][1] - m[1][2]) / s
            x = 0.25 * s
            y = (m[0][1] + m[1][0]) / s
            z = (m[0][2] + m[2][0]) / s
        elif m[1][1] > m[2][2]:
            s = 2.0 * math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
            w = (m[0][2] - m[2][0]) / s
            x = (m[0][1] + m[1][0]) / s
            y = 0.25 * s
            z = (m[1][2] + m[2][1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
            w = (m[1][0] - m[0][1]) / s
            x = (m[0][2] + m[2][0]) / s
            y = (m[1][2] + m[2][1]) / s
            z = 0.25 * s
        return x, y, z, w
