"""Fingertip sensor data → visualization_msgs/MarkerArray conversion.

Visualizes barometer, ToF, force (F), displacement (u), and contact state
from HandSensorState/FingertipSensor messages as RViz markers.
"""

import logging
import math

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA

logger = logging.getLogger('rtc_digital_twin.sensor_visualizer')


class SensorVisualizer:
    """Converts FingertipSensor messages to RViz MarkerArray.

    Per fingertip:
      - Barometer ×8 → Arrow (+Z), pressure-proportional length, heatmap color
      - ToF ×3       → Arrow (+Z), distance-proportional length (default OFF)
      - F ×1         → Arrow (3D direction), force magnitude, red palette
      - u ×1         → Arrow (3D direction), displacement magnitude, cyan palette
      - contact ×1   → Sphere at origin, red (contact) / green (no contact)
    """

    # Barometer sensor positions: 2(X) × 4(Y) grid, 1mm spacing, centered
    BARO_GRID_OFFSETS = [
        (-0.0005, -0.0015, 0.0),  # row 0, col 0
        (-0.0005, -0.0005, 0.0),  # row 0, col 1
        (-0.0005,  0.0005, 0.0),  # row 0, col 2
        (-0.0005,  0.0015, 0.0),  # row 0, col 3
        ( 0.0005, -0.0015, 0.0),  # row 1, col 0
        ( 0.0005, -0.0005, 0.0),  # row 1, col 1
        ( 0.0005,  0.0005, 0.0),  # row 1, col 2
        ( 0.0005,  0.0015, 0.0),  # row 1, col 3
    ]

    # ToF sensor positions (3 sensors along the fingertip X axis)
    TOF_OFFSETS = [
        (-0.005, 0.0, 0.0),
        ( 0.000, 0.0, 0.0),
        ( 0.005, 0.0, 0.0),
    ]

    def __init__(self, fingertip_names, config):
        self.fingertip_names = fingertip_names

        # Barometer config
        self.baro_min = config.get('barometer_min', 0.0)
        self.baro_max = config.get('barometer_max', 1000.0)
        self.baro_arrow_max_length = config.get('barometer_arrow_max_length', 0.015)
        self.baro_arrow_scale = config.get('barometer_arrow_scale', 0.0008)

        # ToF config
        self.tof_enabled = config.get('tof_enabled', False)
        self.tof_max_dist = config.get('tof_max_distance', 0.2)
        self.tof_arrow_scale = config.get('tof_arrow_scale', 0.003)

        # Force config (F max ≈ 10 → 0.003 × 10 = 0.03m = 30mm)
        self.force_arrow_scale = config.get('force_arrow_scale', 0.003)
        self.force_arrow_shaft = config.get('force_arrow_shaft', 0.003)

        # Displacement config (u max ≈ 20 → 0.0015 × 20 = 0.03m = 30mm)
        self.disp_arrow_scale = config.get('displacement_arrow_scale', 0.0015)
        self.disp_arrow_shaft = config.get('displacement_arrow_shaft', 0.002)

        # Contact threshold (same as inferencer: contact_prob < 0.1 → no contact)
        self.contact_threshold = config.get('contact_threshold', 0.1)

        # Contact config
        self.contact_sphere_radius = config.get('contact_sphere_radius', 0.005)

        logger.info(
            'SensorVisualizer initialized: %d fingertips, '
            'baro_range=[%.0f, %.0f], tof=%s',
            len(fingertip_names), self.baro_min, self.baro_max,
            'enabled' if self.tof_enabled else 'disabled')

    def create_markers(self, fingertip_sensors, stamp):
        """Create MarkerArray from FingertipSensor messages.

        Args:
            fingertip_sensors: list of FingertipSensor messages (or None)
            stamp: ROS2 Time message for header stamp
        Returns:
            MarkerArray with all sensor visualizations
        """
        markers = MarkerArray()
        marker_id = 0

        for i, name in enumerate(self.fingertip_names):
            ft = fingertip_sensors[i] if i < len(fingertip_sensors) else None
            if ft is None:
                logger.debug('Fingertip %s: no data', name)
                continue

            frame_id = f'{name}_tip_link'

            # Barometer → +Z Arrow markers
            for j in range(8):
                marker = self._make_baro_arrow(
                    frame_id, marker_id, name, j, ft.barometer[j], stamp)
                markers.markers.append(marker)
                marker_id += 1

            # ToF → +Z Arrow markers (off by default)
            if self.tof_enabled:
                for j in range(3):
                    marker = self._make_tof_arrow(
                        frame_id, marker_id, name, j, ft.tof[j], stamp)
                    markers.markers.append(marker)
                    marker_id += 1

            # Force + Displacement → 3D Arrow (only when contact detected)
            has_contact = (ft.inference_enable
                           and float(ft.contact_flag) >= self.contact_threshold)
            if has_contact:
                marker = self._make_force_arrow(
                    frame_id, marker_id, name, ft.f, stamp)
                markers.markers.append(marker)
                marker_id += 1

                marker = self._make_displacement_arrow(
                    frame_id, marker_id, name, ft.u, stamp)
                markers.markers.append(marker)
                marker_id += 1
            else:
                # Delete stale arrows when contact lost
                for ns_suffix in ('_force', '_displacement'):
                    marker = Marker()
                    marker.header.frame_id = frame_id
                    marker.header.stamp = stamp
                    marker.ns = f'{name}{ns_suffix}'
                    marker.id = marker_id
                    marker.action = Marker.DELETE
                    markers.markers.append(marker)
                    marker_id += 1

            # Contact → Sphere
            marker = self._make_contact_sphere(
                frame_id, marker_id, name,
                ft.contact_flag, ft.inference_enable, stamp)
            markers.markers.append(marker)
            marker_id += 1

        return markers

    # ── Barometer: +Z Arrow ──────────────────────────────────────────────

    def _make_baro_arrow(self, frame_id, marker_id, name, baro_idx, value, stamp):
        """Pressure value → +Z Arrow marker."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = f'{name}_baro'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        ox, oy, oz = self.BARO_GRID_OFFSETS[baro_idx]
        ratio = self._clamp01(
            (value - self.baro_min) / max(self.baro_max - self.baro_min, 1e-9))
        arrow_len = ratio * self.baro_arrow_max_length

        start = Point(x=ox, y=oy, z=oz)
        end = Point(x=ox, y=oy, z=oz + arrow_len)
        marker.points = [start, end]

        s = self.baro_arrow_scale
        marker.scale = Vector3(x=s, y=s * 2.0, z=0.0)
        marker.color = self._heatmap_color(ratio)

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 100_000_000  # 100ms
        return marker

    # ── ToF: +Z Arrow ────────────────────────────────────────────────────

    def _make_tof_arrow(self, frame_id, marker_id, name, tof_idx, distance, stamp):
        """Distance value → +Z Arrow marker."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = f'{name}_tof'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        ox, oy, oz = self.TOF_OFFSETS[tof_idx]
        clamped_dist = min(distance, self.tof_max_dist)

        start = Point(x=ox, y=oy, z=oz)
        end = Point(x=ox, y=oy, z=oz + clamped_dist)
        marker.points = [start, end]

        s = self.tof_arrow_scale
        marker.scale = Vector3(x=s, y=s * 2.0, z=0.0)

        ratio = self._clamp01(clamped_dist / max(self.tof_max_dist, 1e-9))
        marker.color = ColorRGBA(r=1.0 - ratio, g=ratio, b=0.2, a=0.9)

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 100_000_000
        return marker

    # ── Force: 3D Arrow ──────────────────────────────────────────────────

    def _make_force_arrow(self, frame_id, marker_id, name, f, stamp):
        """Force vector → 3D Arrow marker (red palette)."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = f'{name}_force'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        fx, fy, fz = float(f[0]), float(f[1]), float(f[2])
        mag = math.sqrt(fx * fx + fy * fy + fz * fz)

        start = Point(x=0.0, y=0.0, z=0.0)
        if mag > 1e-6:
            length = mag * self.force_arrow_scale
            nx, ny, nz = fx / mag, fy / mag, fz / mag
            end = Point(x=nx * length, y=ny * length, z=nz * length)
        else:
            end = Point(x=0.0, y=0.0, z=0.0)

        marker.points = [start, end]

        s = self.force_arrow_shaft
        marker.scale = Vector3(x=s, y=s * 2.0, z=0.0)
        marker.color = ColorRGBA(r=1.0, g=0.2, b=0.1, a=0.95)

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 100_000_000
        return marker

    # ── Displacement: 3D Arrow ───────────────────────────────────────────

    def _make_displacement_arrow(self, frame_id, marker_id, name, u, stamp):
        """Displacement vector → 3D Arrow marker (cyan palette)."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = f'{name}_displacement'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        ux, uy, uz = float(u[0]), float(u[1]), float(u[2])
        mag = math.sqrt(ux * ux + uy * uy + uz * uz)

        start = Point(x=0.0, y=0.0, z=0.0)
        if mag > 1e-6:
            length = mag * self.disp_arrow_scale
            nx, ny, nz = ux / mag, uy / mag, uz / mag
            end = Point(x=nx * length, y=ny * length, z=nz * length)
        else:
            end = Point(x=0.0, y=0.0, z=0.0)

        marker.points = [start, end]

        s = self.disp_arrow_shaft
        marker.scale = Vector3(x=s, y=s * 2.0, z=0.0)
        marker.color = ColorRGBA(r=0.0, g=0.85, b=0.9, a=0.95)

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 100_000_000
        return marker

    # ── Contact: Sphere ──────────────────────────────────────────────────

    def _make_contact_sphere(self, frame_id, marker_id, name,
                             contact_flag, inference_enable, stamp):
        """Contact probability → colored Sphere at fingertip origin."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = f'{name}_contact'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        d = self.contact_sphere_radius * 2.0
        marker.scale = Vector3(x=d, y=d, z=d)

        if not inference_enable:
            # Grey when inference not ready (calibrating)
            marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.3)
        else:
            prob = self._clamp01(float(contact_flag))
            marker.color = ColorRGBA(
                r=prob, g=1.0 - prob, b=0.0, a=0.5 + 0.3 * prob)

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 100_000_000
        return marker

    # ── Utilities ────────────────────────────────────────────────────────

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
