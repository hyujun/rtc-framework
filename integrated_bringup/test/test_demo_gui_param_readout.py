"""Unit tests for the demo GUI's cached parameter reader (#852).

The Grasp tab's mode readout and the Virtual TCP panel read their parameters
through one ``CachedParamReadout``; #852 was the two keeping separate copies of
these rules and the grasp copy falling behind. Every test here runs over both
panels' specs, so a rule that holds for one and not the other cannot pass.

The panel-side halves (what a paint does to the widgets, the send path, a poll
landing while a forced read is in flight) stay in ``test_demo_gui_grasp_mode``
and ``test_demo_gui_vtcp``.
"""

from __future__ import annotations

import types

import pytest
from rcl_interfaces.msg import ParameterType

from integrated_bringup.demo_gui.config import (
    GRASP_MODE_OWNERS,
    GRASP_MODE_PARAM,
    GRASP_MODE_READOUT,
    GRASP_MODE_UNKNOWN,
)
from integrated_bringup.demo_gui.param_readout import CachedParamReadout
from integrated_bringup.demo_gui.vtcp import (
    VTCP_OWNERS,
    VTCP_PARAMS,
    VTCP_READOUT,
    VtcpConfigView,
)

WBC = "demo_wbc_controller"  # declares neither panel's parameters


def _value(type_: int, string_value: str = "", double_array_value=()):
    return types.SimpleNamespace(
        type=type_, string_value=string_value, double_array_value=list(double_array_value)
    )


_STRING = ParameterType.PARAMETER_STRING
_DOUBLE_ARRAY = ParameterType.PARAMETER_DOUBLE_ARRAY

_VIEW = VtcpConfigView("constant", (0.0, 0.0, 0.1), (0.0, 0.0, 0.0))

# (spec, an owner, a good answer, its parsed value, a different parsed value,
#  an answer that does not parse)
_CASES = [
    pytest.param(
        GRASP_MODE_READOUT,
        "demo_joint_controller",
        [_value(_STRING, string_value="force_pi")],
        "force_pi",
        "contact_stop",
        # PARAMETER_NOT_SET: a controller older than the read-only mirror.
        [_value(ParameterType.PARAMETER_NOT_SET)],
        id="grasp_mode",
    ),
    pytest.param(
        VTCP_READOUT,
        "demo_task_controller",
        [
            _value(_STRING, string_value="constant"),
            _value(_DOUBLE_ARRAY, double_array_value=(0.0, 0.0, 0.1)),
            _value(_DOUBLE_ARRAY, double_array_value=(0.0, 0.0, 0.0)),
        ],
        _VIEW,
        VtcpConfigView("disabled", (0.0,) * 3, (0.0,) * 3),
        # A controller older than #847: nothing declared.
        [_value(ParameterType.PARAMETER_NOT_SET)] * 3,
        id="vtcp",
    ),
]
_ARGS = "spec, owner, answer, value, other, unreadable"


class _Future:
    def __init__(self, answer, raises):
        self._answer = answer
        self._raises = raises
        self.callbacks = []

    def result(self):
        if self._raises is not None:
            raise self._raises
        return types.SimpleNamespace(values=self._answer)

    def add_done_callback(self, cb):
        # Held until the test lands the answer, as the executor would later.
        self.callbacks.append(cb)

    def land(self):
        for cb in self.callbacks:
            cb(self)


class _Client:
    def __init__(self, ready, answer, raises):
        self._ready = ready
        self._answer = answer
        self._raises = raises
        self.requests = []
        self.futures = []

    def services_are_ready(self):
        return self._ready

    def wait_for_services(self, timeout_sec=0.0):
        raise AssertionError("blocks the Tk thread for up to the timeout")

    def get_parameters(self, names):
        self.requests.append(names)
        future = _Future(self._answer, self._raises)
        self.futures.append(future)
        return future


class _Logger:
    def __init__(self):
        self.lines = []

    def __getattr__(self, level):
        return lambda msg: self.lines.append((level, msg))


def _reader(spec, *, ready=True, answer=None, raises=None):
    painted = []
    deferred = []
    client = _Client(ready, answer, raises)
    logger = _Logger()
    reader = CachedParamReadout(
        spec,
        {},
        get_client=lambda _ctrl: client,
        paint=lambda c, v: painted.append((c, v)),
        defer=lambda fn, *args: deferred.append((fn, args)),
        logger=lambda: logger,
    )
    return reader, client, painted, deferred


def test_each_spec_covers_its_panels_full_owner_set():
    """config.py extends GRASP_MODE_OWNERS after its first definition; a spec
    built before that line would hold the shorter set and never query the
    controller added later."""
    assert GRASP_MODE_READOUT.owners == GRASP_MODE_OWNERS
    assert GRASP_MODE_READOUT.params == (GRASP_MODE_PARAM,)
    assert GRASP_MODE_READOUT.unknown == GRASP_MODE_UNKNOWN
    assert VTCP_READOUT.owners == VTCP_OWNERS
    assert VTCP_READOUT.params == VTCP_PARAMS
    assert VTCP_READOUT.unknown is None


@pytest.mark.parametrize(_ARGS, _CASES)
def test_a_cache_hit_paints_without_a_round_trip(spec, owner, answer, value, other, unreadable):
    """A cache hit must not reach the parameter service — that is what keeps the
    5 s catalog poll and every radio click off it."""
    reader, client, painted, _ = _reader(spec, answer=answer)
    reader.cache[owner] = value
    reader.refresh(owner)

    assert painted == [(owner, value)]
    assert client.requests == []


@pytest.mark.parametrize(_ARGS, _CASES)
def test_a_non_owner_is_painted_unknown_and_not_queried(
    spec, owner, answer, value, other, unreadable
):
    reader, client, painted, _ = _reader(spec, answer=answer)
    reader.refresh(WBC)

    assert painted == [(WBC, spec.unknown)]
    assert client.requests == []


@pytest.mark.parametrize(_ARGS, _CASES)
def test_a_miss_paints_unknown_and_caches_nothing_while_services_are_down(
    spec, owner, answer, value, other, unreadable
):
    """Two properties at once: the panel drops to unknown rather than keep
    showing the previous controller's value while a read is pending, and a
    read that never happened is not cached — that is what makes the next
    catalog poll retry once the CM comes up."""
    reader, client, painted, _ = _reader(spec, ready=False, answer=answer)
    reader.refresh(owner)

    assert painted == [(owner, spec.unknown)]
    assert client.requests == []
    assert reader.cache == {}


@pytest.mark.parametrize(_ARGS, _CASES)
def test_a_forced_refresh_paints_nothing_until_its_answer_lands(
    spec, owner, answer, value, other, unreadable
):
    """Neither the cached value (the one being re-checked) nor unknown, which
    would blank the panel's request widgets. The cache stays, so a poll that
    lands before the answer repaints it."""
    reader, client, painted, deferred = _reader(spec, answer=answer)
    reader.cache[owner] = other
    reader.refresh(owner, force=True)

    assert painted == []
    assert client.requests == [list(spec.params)]
    reader.refresh(owner)  # the poll
    assert painted == [(owner, other)]

    client.futures[0].land()
    assert reader.cache == {owner: value}
    assert deferred == [(reader._paint, (owner, value))], "the answer must reach Tk via defer"


@pytest.mark.parametrize(_ARGS, _CASES)
def test_an_answer_is_cached_and_painted_on_the_tk_thread(
    spec, owner, answer, value, other, unreadable
):
    reader, client, painted, deferred = _reader(spec, answer=answer)
    reader.refresh(owner)
    assert painted == [(owner, spec.unknown)]

    client.futures[0].land()
    assert reader.cache == {owner: value}
    assert painted == [(owner, spec.unknown)], "painted from the executor thread"
    fn, args = deferred[0]
    fn(*args)
    assert painted[-1] == (owner, value)


@pytest.mark.parametrize(_ARGS, _CASES)
def test_an_unreadable_answer_is_cached_as_nothing(spec, owner, answer, value, other, unreadable):
    reader, client, _, deferred = _reader(spec, answer=unreadable)
    reader.refresh(owner)
    client.futures[0].land()

    assert reader.cache == {}
    assert deferred == []


@pytest.mark.parametrize(_ARGS, _CASES)
def test_a_failed_forced_read_stops_claiming_the_cached_value(
    spec, owner, answer, value, other, unreadable
):
    """A forced read follows a change, so the cached value may be the one that
    changed."""
    reader, client, _, deferred = _reader(spec, raises=RuntimeError("service died"))
    reader.cache[owner] = other
    reader.refresh(owner, force=True)
    client.futures[0].land()

    assert reader.cache == {}
    assert deferred == []


@pytest.mark.parametrize(_ARGS, _CASES)
def test_an_empty_controller_key_is_ignored(spec, owner, answer, value, other, unreadable):
    reader, client, painted, _ = _reader(spec, answer=answer)
    reader.refresh("")
    reader.refresh("", force=True)

    assert painted == []
    assert client.requests == []
