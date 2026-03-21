"""Fingertip sensor data → visualization_msgs/MarkerArray conversion."""

import math

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA


class SensorVisualizer:
    """Converts fingertip sensor data (barometer + ToF) to RViz MarkerArray.

    Each fingertip has 8 barometer sensors and 3 ToF sensors (11 total).
    - Barometer → Sphere markers (size & color proportional to pressure)
    - ToF → Arrow markers (length proportional to distance)
    """

    # Barometer sensor positions relative to fingertip frame
    # Arranged in a 2×4 grid on the fingertip pad surface
    BARO_GRID_OFFSETS = [
        (-0.004, -0.006, 0.0),  # row 0, col 0
        (-0.004, -0.002, 0.0),  # row 0, col 1
        (-0.004,  0.002, 0.0),  # row 0, col 2
        (-0.004,  0.006, 0.0),  # row 0, col 3
        ( 0.004, -0.006, 0.0),  # row 1, col 0
        ( 0.004, -0.002, 0.0),  # row 1, col 1
        ( 0.004,  0.002, 0.0),  # row 1, col 2
        ( 0.004,  0.006, 0.0),  # row 1, col 3
    ]

    # ToF sensor positions (3 sensors along the fingertip)
    TOF_OFFSETS = [
        (-0.005, 0.0, 0.0),
        ( 0.000, 0.0, 0.0),
        ( 0.005, 0.0, 0.0),
    ]

    def __init__(self, fingertip_names, config):
        self.fingertip_names = fingertip_names
        self.baro_min = config.get('barometer_min', 0.0)
        self.baro_max = config.get('barometer_max', 1000.0)
        self.sphere_min = config.get('barometer_sphere_min', 0.002)
        self.sphere_max = config.get('barometer_sphere_max', 0.008)
        self.tof_max_dist = config.get('tof_max_distance', 0.2)
        self.tof_arrow_scale = config.get('tof_arrow_scale', 0.003)

    def create_markers(self, fingertip_sensors, stamp):
        """Create MarkerArray from fingertip sensor data.

        Args:
            fingertip_sensors: list[4][11] — [baro×8, tof×3] per fingertip
            stamp: ROS2 Time message for header stamp
        Returns:
            MarkerArray with barometer spheres and ToF arrows
        """
        markers = MarkerArray()
        marker_id = 0

        for i, (name, sensors) in enumerate(
            zip(self.fingertip_names, fingertip_sensors)
        ):
            baro = sensors[0:8]
            tof = sensors[8:11]

            # Barometer → Sphere markers
            for j, pressure in enumerate(baro):
                marker = self._make_baro_sphere(name, marker_id, j, pressure, stamp)
                markers.markers.append(marker)
                marker_id += 1

            # ToF → Arrow markers
            for j, distance in enumerate(tof):
                marker = self._make_tof_arrow(name, marker_id, j, distance, stamp)
                markers.markers.append(marker)
                marker_id += 1

        return markers

    def _make_baro_sphere(self, fingertip_name, marker_id, baro_idx, value, stamp):
        """Pressure value → sized & colored Sphere marker."""
        marker = Marker()
        marker.header.frame_id = f'{fingertip_name}_tip_link'
        marker.header.stamp = stamp
        marker.ns = f'{fingertip_name}_baro'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position from grid layout
        ox, oy, oz = self.BARO_GRID_OFFSETS[baro_idx]
        marker.pose.position.x = ox
        marker.pose.position.y = oy
        marker.pose.position.z = oz
        marker.pose.orientation.w = 1.0

        # Size: proportional to pressure
        ratio = self._clamp01((value - self.baro_min) / max(self.baro_max - self.baro_min, 1e-9))
        diameter = 2.0 * (self.sphere_min + ratio * (self.sphere_max - self.sphere_min))
        marker.scale = Vector3(x=diameter, y=diameter, z=diameter)

        # Color: heatmap
        marker.color = self._heatmap_color(ratio)

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 100_000_000  # 100ms

        return marker

    def _make_tof_arrow(self, fingertip_name, marker_id, tof_idx, distance, stamp):
        """Distance value → Arrow marker pointing away from fingertip."""
        marker = Marker()
        marker.header.frame_id = f'{fingertip_name}_tip_link'
        marker.header.stamp = stamp
        marker.ns = f'{fingertip_name}_tof'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Arrow defined by start/end points
        ox, oy, oz = self.TOF_OFFSETS[tof_idx]
        clamped_dist = min(distance, self.tof_max_dist)

        start = Point(x=ox, y=oy, z=oz)
        end = Point(x=ox, y=oy, z=oz + clamped_dist)
        marker.points = [start, end]

        # Arrow shaft and head size
        s = self.tof_arrow_scale
        marker.scale = Vector3(x=s, y=s * 2.0, z=0.0)

        # Color: distance-based (red=close, green=far)
        ratio = self._clamp01(clamped_dist / max(self.tof_max_dist, 1e-9))
        marker.color = ColorRGBA(
            r=1.0 - ratio,
            g=ratio,
            b=0.2,
            a=0.9,
        )

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 100_000_000  # 100ms

        return marker

    @staticmethod
    def _heatmap_color(ratio):
        """0.0–1.0 → RGBA heatmap (blue → green → red)."""
        r = max(0.0, min(1.0, 2.0 * ratio - 1.0))
        g = 1.0 - abs(2.0 * ratio - 1.0)
        b = max(0.0, min(1.0, 1.0 - 2.0 * ratio))
        return ColorRGBA(r=r, g=g, b=b, a=0.9)

    @staticmethod
    def _clamp01(v):
        return max(0.0, min(1.0, v))
