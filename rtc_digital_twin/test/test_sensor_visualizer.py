"""Tests for SensorVisualizer — FingertipSensor → MarkerArray conversion.

Pure conversion logic: needs ROS message types (visualization/geometry/std_msgs)
but no ROS node, spin, or publisher. FingertipSensor inputs are duck-typed with
a lightweight fake so the tests exercise the marker math directly.
"""

import math

import pytest
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

from rtc_digital_twin.sensor_visualizer import SensorVisualizer

# ── Fakes ───────────────────────────────────────────────────────────────────


class FakeFingertip:
    """Duck-typed stand-in for rtc_msgs/FingertipSensor."""

    def __init__(
        self,
        *,
        barometer=None,
        tof=None,
        f=(0.0, 0.0, 0.0),
        u=(0.0, 0.0, 0.0),
        contact_flag=0.0,
        inference_enable=False,
    ):
        self.barometer = list(barometer) if barometer is not None else [0.0] * 8
        self.tof = list(tof) if tof is not None else [0.0] * 3
        self.f = list(f)
        self.u = list(u)
        self.contact_flag = contact_flag
        self.inference_enable = inference_enable


def _stamp():
    return Time(sec=0, nanosec=0)


# ── Static utilities ─────────────────────────────────────────────────────────


class TestHeatmapColor:
    def test_low_is_blue(self):
        c = SensorVisualizer._heatmap_color(0.0)
        assert (c.r, c.g, c.b) == pytest.approx((0.0, 0.0, 1.0))
        assert c.a == pytest.approx(0.9)

    def test_mid_is_green(self):
        c = SensorVisualizer._heatmap_color(0.5)
        assert (c.r, c.g, c.b) == pytest.approx((0.0, 1.0, 0.0))

    def test_high_is_red(self):
        c = SensorVisualizer._heatmap_color(1.0)
        assert (c.r, c.g, c.b) == pytest.approx((1.0, 0.0, 0.0))

    def test_quarter_blend(self):
        c = SensorVisualizer._heatmap_color(0.25)
        assert (c.r, c.g, c.b) == pytest.approx((0.0, 0.5, 0.5))


class TestClamp01:
    def test_below_zero(self):
        assert SensorVisualizer._clamp01(-3.0) == 0.0

    def test_above_one(self):
        assert SensorVisualizer._clamp01(2.5) == 1.0

    def test_inside(self):
        assert SensorVisualizer._clamp01(0.4) == pytest.approx(0.4)


# ── create_markers: counts and branch coverage ───────────────────────────────


class TestCreateMarkers:
    def _viz(self, names=("index",), **cfg):
        return SensorVisualizer(list(names), dict(cfg))

    def test_no_data_yields_empty(self):
        viz = self._viz()
        out = viz.create_markers([None], _stamp())
        assert out.markers == []

    def test_short_sensor_list_skips_missing(self):
        # 2 names, 1 sensor → second fingertip has no data and is skipped.
        viz = self._viz(names=("index", "thumb"))
        out = viz.create_markers([FakeFingertip()], _stamp())
        # 8 baro + 2 DELETE (no contact) + 1 contact sphere = 11
        assert len(out.markers) == 11

    def test_no_contact_emits_delete_arrows(self):
        viz = self._viz()
        out = viz.create_markers([FakeFingertip(inference_enable=False)], _stamp())
        assert len(out.markers) == 11
        deletes = [m for m in out.markers if m.action == Marker.DELETE]
        assert len(deletes) == 2
        assert {m.ns for m in deletes} == {"index_force", "index_displacement"}

    def test_contact_emits_force_and_displacement(self):
        viz = self._viz()
        ft = FakeFingertip(inference_enable=True, contact_flag=1.0, f=(1.0, 0.0, 0.0))
        out = viz.create_markers([ft], _stamp())
        # 8 baro + force + displacement + contact sphere = 11 (no DELETEs)
        assert len(out.markers) == 11
        assert all(m.action == Marker.ADD for m in out.markers)
        nss = {m.ns for m in out.markers}
        assert "index_force" in nss
        assert "index_displacement" in nss

    def test_contact_below_threshold_treated_as_no_contact(self):
        viz = self._viz(contact_threshold=0.5)
        ft = FakeFingertip(inference_enable=True, contact_flag=0.4)
        out = viz.create_markers([ft], _stamp())
        deletes = [m for m in out.markers if m.action == Marker.DELETE]
        assert len(deletes) == 2

    def test_tof_enabled_adds_three_arrows(self):
        viz = self._viz(tof_enabled=True)
        out = viz.create_markers([FakeFingertip()], _stamp())
        # 8 baro + 3 tof + 2 DELETE + 1 contact = 14
        assert len(out.markers) == 14
        assert sum(1 for m in out.markers if m.ns == "index_tof") == 3

    def test_frame_id_equals_name_verbatim(self):
        # Marker frame_id is the fingertip name used as-is (no suffix), so a
        # bringup yaml carries the full URDF link name.
        viz = self._viz(names=("l_index_tip_bracket",))
        out = viz.create_markers([FakeFingertip()], _stamp())
        assert out.markers  # sanity: markers were emitted
        assert {m.header.frame_id for m in out.markers} == {"l_index_tip_bracket"}

    def test_two_fingertips(self):
        viz = self._viz(names=("index", "thumb"))
        out = viz.create_markers([FakeFingertip(), FakeFingertip()], _stamp())
        assert len(out.markers) == 22

    def test_marker_ids_are_unique(self):
        viz = self._viz(names=("index", "thumb"), tof_enabled=True)
        out = viz.create_markers([FakeFingertip(), FakeFingertip()], _stamp())
        ids = [m.id for m in out.markers]
        assert ids == list(range(len(ids)))


# ── Force-magnitude fallback gate ─────────────────────────────────────────────


class TestForceFallback:
    """create_markers force_display_threshold gate (classifier-less robots)."""

    def _viz(self, names=("index",), **cfg):
        return SensorVisualizer(list(names), dict(cfg))

    @staticmethod
    def _ns(markers, ns):
        return [m for m in markers if m.ns == ns]

    def test_force_fallback_disabled_by_default(self):
        # No threshold set (0.0): force alone must not open the gate, even with
        # inference enabled — proves the existing contact_flag behavior is kept.
        viz = self._viz()
        ft = FakeFingertip(inference_enable=True, contact_flag=0.0, f=(0.0, 0.0, 5.0))
        out = viz.create_markers([ft], _stamp())
        force = self._ns(out.markers, "index_force")
        disp = self._ns(out.markers, "index_displacement")
        assert [m.action for m in force] == [Marker.DELETE]
        assert [m.action for m in disp] == [Marker.DELETE]

    def test_force_fallback_renders_above_threshold(self):
        viz = self._viz(force_display_threshold=1.0)
        ft = FakeFingertip(inference_enable=True, contact_flag=0.0, f=(0.0, 0.0, 2.0))
        out = viz.create_markers([ft], _stamp())
        force = self._ns(out.markers, "index_force")
        assert [m.action for m in force] == [Marker.ADD]

    def test_force_fallback_below_threshold_deletes(self):
        viz = self._viz(force_display_threshold=1.0)
        ft = FakeFingertip(inference_enable=True, contact_flag=0.0, f=(0.0, 0.0, 0.5))
        out = viz.create_markers([ft], _stamp())
        force = self._ns(out.markers, "index_force")
        assert [m.action for m in force] == [Marker.DELETE]

    def test_force_fallback_requires_inference_enable(self):
        # inference_enable False → gate stays closed regardless of |F|; sphere grey.
        viz = self._viz(force_display_threshold=1.0)
        ft = FakeFingertip(inference_enable=False, contact_flag=0.0, f=(0.0, 0.0, 5.0))
        out = viz.create_markers([ft], _stamp())
        force = self._ns(out.markers, "index_force")
        assert [m.action for m in force] == [Marker.DELETE]
        sphere = self._ns(out.markers, "index_contact")[0]
        assert (sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a) == pytest.approx(
            (0.5, 0.5, 0.5, 0.3)
        )

    def test_fallback_zero_displacement_emits_delete(self):
        # Fallback ADD path with u ≡ 0 → force ADD but displacement DELETE
        # (no zero-length arrow); id sequence stays unique/contiguous.
        viz = self._viz(force_display_threshold=1.0)
        ft = FakeFingertip(
            inference_enable=True, contact_flag=0.0, f=(0.0, 0.0, 2.0), u=(0.0, 0.0, 0.0)
        )
        out = viz.create_markers([ft], _stamp())
        force = self._ns(out.markers, "index_force")[0]
        disp = self._ns(out.markers, "index_displacement")[0]
        assert force.action == Marker.ADD
        assert disp.action == Marker.DELETE
        ids = [m.id for m in out.markers]
        assert ids == list(range(len(ids)))

    def test_fallback_contact_sphere_red(self):
        # Fallback active but contact_flag 0 → sphere forced red for consistency.
        viz = self._viz(force_display_threshold=1.0)
        ft = FakeFingertip(inference_enable=True, contact_flag=0.0, f=(0.0, 0.0, 2.0))
        out = viz.create_markers([ft], _stamp())
        sphere = self._ns(out.markers, "index_contact")[0]
        assert (sphere.color.r, sphere.color.g) == pytest.approx((1.0, 0.0))


# ── Marker geometry (private builders) ────────────────────────────────────────


class TestBaroArrow:
    def _viz(self):
        return SensorVisualizer(["index"], {"barometer_min": 0.0, "barometer_max": 1000.0})

    def test_max_pressure_full_length_red(self):
        viz = self._viz()
        m = viz._make_baro_arrow("f", 0, "index", 0, 1000.0, _stamp())
        ox, oy, oz = SensorVisualizer.BARO_GRID_OFFSETS[0]
        assert m.points[1].z == pytest.approx(oz + viz.baro_arrow_max_length)
        assert (m.color.r, m.color.g, m.color.b) == pytest.approx((1.0, 0.0, 0.0))

    def test_min_pressure_zero_length_blue(self):
        viz = self._viz()
        m = viz._make_baro_arrow("f", 0, "index", 3, 0.0, _stamp())
        ox, oy, oz = SensorVisualizer.BARO_GRID_OFFSETS[3]
        assert m.points[1].z == pytest.approx(oz)
        assert (m.color.r, m.color.g, m.color.b) == pytest.approx((0.0, 0.0, 1.0))

    def test_value_above_max_clamped(self):
        viz = self._viz()
        m = viz._make_baro_arrow("f", 0, "index", 0, 5000.0, _stamp())
        ox, oy, oz = SensorVisualizer.BARO_GRID_OFFSETS[0]
        # ratio clamped to 1.0 → length capped at baro_arrow_max_length
        assert m.points[1].z == pytest.approx(oz + viz.baro_arrow_max_length)


class TestTofArrow:
    def test_within_range(self):
        viz = SensorVisualizer(["index"], {"tof_max_distance": 0.2})
        m = viz._make_tof_arrow("f", 0, "index", 0, 0.1, _stamp())
        ox, oy, oz = SensorVisualizer.TOF_OFFSETS[0]
        assert m.points[1].z == pytest.approx(oz + 0.1)
        # ratio 0.5 → r=0.5, g=0.5, b=0.2
        assert (m.color.r, m.color.g, m.color.b) == pytest.approx((0.5, 0.5, 0.2))

    def test_distance_clamped_to_max(self):
        viz = SensorVisualizer(["index"], {"tof_max_distance": 0.2})
        m = viz._make_tof_arrow("f", 1, "index", 1, 0.9, _stamp())
        ox, oy, oz = SensorVisualizer.TOF_OFFSETS[1]
        assert m.points[1].z == pytest.approx(oz + 0.2)
        assert (m.color.r, m.color.g) == pytest.approx((0.0, 1.0))


class TestForceArrow:
    def test_direction_and_length(self):
        viz = SensorVisualizer(["index"], {"force_arrow_scale": 0.003})
        m = viz._make_force_arrow("f", 0, "index", (3.0, 0.0, 0.0), _stamp())
        assert m.points[0].x == 0.0
        assert m.points[1].x == pytest.approx(3.0 * 0.003)
        assert m.points[1].y == pytest.approx(0.0)
        assert (m.color.r, m.color.g, m.color.b, m.color.a) == pytest.approx((1.0, 0.2, 0.1, 0.95))

    def test_zero_force_collapses_to_origin(self):
        viz = SensorVisualizer(["index"], {})
        m = viz._make_force_arrow("f", 0, "index", (0.0, 0.0, 0.0), _stamp())
        assert (m.points[1].x, m.points[1].y, m.points[1].z) == (0.0, 0.0, 0.0)

    def test_normalization(self):
        viz = SensorVisualizer(["index"], {"force_arrow_scale": 1.0})
        m = viz._make_force_arrow("f", 0, "index", (0.0, 3.0, 4.0), _stamp())
        end = m.points[1]
        length = math.sqrt(end.x**2 + end.y**2 + end.z**2)
        assert length == pytest.approx(5.0)  # mag(0,3,4)=5 × scale 1.0


class TestDisplacementArrow:
    def test_direction_and_length(self):
        viz = SensorVisualizer(["index"], {"displacement_arrow_scale": 0.0015})
        m = viz._make_displacement_arrow("f", 0, "index", (0.0, 4.0, 0.0), _stamp())
        assert m.points[1].y == pytest.approx(4.0 * 0.0015)
        assert (m.color.r, m.color.g, m.color.b) == pytest.approx((0.0, 0.85, 0.9))

    def test_zero_collapses_to_origin(self):
        viz = SensorVisualizer(["index"], {})
        m = viz._make_displacement_arrow("f", 0, "index", (0.0, 0.0, 0.0), _stamp())
        assert (m.points[1].x, m.points[1].y, m.points[1].z) == (0.0, 0.0, 0.0)


class TestContactSphere:
    def test_calibrating_is_grey(self):
        viz = SensorVisualizer(["index"], {"contact_sphere_radius": 0.005})
        m = viz._make_contact_sphere("f", 0, "index", 1.0, False, _stamp())
        assert (m.color.r, m.color.g, m.color.b, m.color.a) == pytest.approx((0.5, 0.5, 0.5, 0.3))
        assert m.scale.x == pytest.approx(0.01)

    def test_full_contact_is_red(self):
        viz = SensorVisualizer(["index"], {})
        m = viz._make_contact_sphere("f", 0, "index", 1.0, True, _stamp())
        assert (m.color.r, m.color.g) == pytest.approx((1.0, 0.0))
        assert m.color.a == pytest.approx(0.8)

    def test_partial_contact_blends(self):
        viz = SensorVisualizer(["index"], {})
        m = viz._make_contact_sphere("f", 0, "index", 0.5, True, _stamp())
        assert (m.color.r, m.color.g) == pytest.approx((0.5, 0.5))
        assert m.color.a == pytest.approx(0.65)
