"""
catching_keys.py — #711: the catching controller's renamed YAML keys and mirror names

Two tables, both generated from one key map (the C++ side owns the first as
``kRenamedCatchingKeys`` in ``rtc_controllers/catching/catching_params.hpp``;
``test_catching_keys.py`` pins this copy equal to the header).

* :data:`RENAMED_CATCHING_KEYS` — the 18 YAML paths (below ``catching:``) that
  moved. A profile that still writes one is refused by :func:`reject_renamed_keys`,
  because a reader that looks only at the new path would take its default and
  never notice the old value (the same refusal the controller makes at configure).
* :data:`RENAMED_MIRRORS` — the 45 read-only ROS parameter names that moved
  with them. Sessions recorded before the rename keep the old names in
  ``mirror.txt`` / ``run_meta.json``; :func:`normalize_mirror` reads either name
  and refuses a record that carries both an old name and its new name.
* :data:`RENAMED_COLUMNS` — the 49 columns of ``catching_diag.csv`` /
  ``planner_events.csv`` that moved (``decel_*`` → ``segment_*``). Readers use the new
  names and load a recorded file through :func:`read_csv_normalized` /
  :func:`column_renames`; a header that mixes old and new names is refused.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping

# (old path under `catching:`, new path). `supervisor.decel` is judged leaf by leaf:
# `supervisor.decel.a_dec` stays where it is.
_GRID = (
    "budget_s",
    "max_ik",
    "n_settle",
    "slice",
    "time",
    "unc",
    "gamma",
    "rollout",
    "budget",
    "score",
    "workspace",
    "hand",
    "ik",
    "catchability",
    "switch",
)
RENAMED_CATCHING_KEYS: tuple[tuple[str, str], ...] = (
    ("supervisor.decel.mode", "planner.segment.mode"),
    ("supervisor.decel.switch_margin", "planner.segment.mpc.switch_margin"),
    ("planner.decel_mpc", "planner.segment.mpc"),
    *((f"planner.{k}", f"planner.search.grid.{k}") for k in _GRID),
)

# Leaves of `planner.decel_mpc` the controller mirrors as read-only parameters.
_MPC_MIRRORED = (
    "horizon.n_nodes",
    "horizon.dt_s",
    "horizon.blocks",
    "m_q",
    "replan.k_max",
    "eta_tau",
    "publish.slack_max",
    "publish.slack_terminal_max",
    "approach.n_pre_max",
    "approach.dt_pre_s",
    "approach.rest_tol",
    "budget.first_s",
    "budget.replan_s",
    "replan.same_point",
    "publish.catch_pos_err_max",
    "catch.w_axis",
    "catch.w_v_par",
    "catch.w_v_perp",
    "catch.gamma_ref",
    "catch.kappa",
    "catch.sigma_floor",
    "catch.w_max",
    "catch.w_const",
    "catch.sigma_ref",
    "catch.rho_v",
    "catch.v_rel_allow",
    "cost.jerk_weight",
    "cost.u_scale",
    "cost.w_delta",
    "cost.rho_tau",
    "cost.w_perp",
    "catch.axis_theta_max",
    "linearization.delta_tr",
    "linearization.reference_rest_tol",
    "linearization.ref_speed_fraction",
    "solver.max_iter",
    "solver.max_iter_in",
    "solver.eps_abs",
    "solver.eps_rel",
)

# old mirror name -> new mirror name (45).
RENAMED_MIRRORS: dict[str, str] = {
    **{f"planner.decel_mpc.{k}": f"planner.segment.mpc.{k}" for k in _MPC_MIRRORED},
    "supervisor.decel.mode": "planner.segment.mode",
    "supervisor.decel.switch_margin": "planner.segment.mpc.switch_margin",
    "planner.gamma.eta_v": "planner.search.grid.gamma.eta_v",
    "planner.time.margin": "planner.search.grid.time.margin",
    "planner.slice.dt": "planner.search.grid.slice.dt",
    "planner.slice.t_lead_min": "planner.search.grid.slice.t_lead_min",
}
_NEW_TO_OLD = {new: old for old, new in RENAMED_MIRRORS.items()}


class RefusedCatchingInput(SystemExit):
    """An input this package refuses because of the names it uses.

    A ``SystemExit`` on purpose, like the other user-input errors of the analysis tools
    (``raise SystemExit("missing …")``): a tool that does not catch it ends with the
    message on one line and exit status 1, not with a traceback.
    """


class RenamedCatchingKeyError(RefusedCatchingInput):
    """A catching profile that still writes a key that moved."""


class MixedMirrorNamesError(RefusedCatchingInput):
    """A recorded mirror that holds an old name together with a new one."""


def _has_path(node: object, path: str) -> bool:
    for part in path.split("."):
        if not isinstance(node, Mapping) or part not in node:
            return False
        node = node[part]
    return True


def find_renamed_keys(catching: object) -> list[tuple[str, str]]:
    """The ``(old, new)`` entries whose old path is written in ``catching`` — with any
    value, a map or a null included. A node on the way that is no map has no children."""
    return [(old, new) for old, new in RENAMED_CATCHING_KEYS if _has_path(catching, old)]


def reject_renamed_keys(catching: object, *, source: str = "catching profile") -> None:
    """Raise :class:`RenamedCatchingKeyError` naming every old key ``catching`` still
    writes and the path it moved to. ``catching`` is the tree under ``catching:``
    (``None`` / a non-map is accepted: nothing to refuse)."""
    found = find_renamed_keys(catching)
    if found:
        lines = "; ".join(f"catching.{old} → catching.{new}" for old, new in found)
        raise RenamedCatchingKeyError(
            f"{source}: renamed catching key(s) still written ({lines}) — "
            "rename them (#711); the old paths are no longer read"
        )


def mirror_name(name: str) -> str:
    """The current name of a mirror parameter (an old or a current name in)."""
    return RENAMED_MIRRORS.get(name, name)


def normalize_mirror(mirror: Mapping, *, source: str = "mirror") -> dict:
    """``mirror`` with every old name renamed to the new one.

    A record that holds an old name together with a new one — of the same key or
    of another — is refused (:class:`MixedMirrorNamesError`): a controller writes
    either all old names or all new ones, so no controller wrote it, and which of
    its values are stale cannot be told.
    """
    old = sorted(name for name in mirror if name in RENAMED_MIRRORS)
    new = sorted(name for name in mirror if name in _NEW_TO_OLD)
    if old and new:
        raise MixedMirrorNamesError(
            f"{source}: holds old mirror names ({', '.join(old)}) together with new ones "
            f"({', '.join(new)})"
        )
    return {mirror_name(name): value for name, value in mirror.items()}


def normalize_run_meta(meta: Mapping, *, source: str = "run_meta.json") -> dict:
    """``meta`` (a ``run_meta.json``) with its ``controller_mirror`` normalized by
    :func:`normalize_mirror`; every other key is untouched."""
    out = dict(meta)
    mirror = out.get("controller_mirror")
    if isinstance(mirror, Mapping):
        out["controller_mirror"] = normalize_mirror(mirror, source=f"{source} controller_mirror")
    return out


# ── The two controller CSVs' renamed columns ────────────────────────────────
# old column -> new column (49: `decel_seq` is one name in both files). The segment
# lane's columns of ``catching_diag.csv`` (17) and ``planner_events.csv`` (33) said
# `decel`; sessions recorded before the rename keep the old names. A reader works with
# the new names and normalizes at the point it loads the file (:func:`column_renames`).
# ``test_catching_keys.py`` pins the new names to what the C++ headers write.
RENAMED_COLUMNS: dict[str, str] = {
    "decel_judged": "segment_judged",
    "decel_refusal": "segment_refusal",
    "decel_event": "segment_event",
    "decel_following": "segment_following",
    "decel_seq": "segment_seq",
    "decel_k0": "segment_k0",
    "decel_held": "segment_held",
    "decel_p_d_x": "segment_p_d_x",
    "decel_p_d_y": "segment_p_d_y",
    "decel_p_d_z": "segment_p_d_z",
    "decel_v_ff_x": "segment_v_ff_x",
    "decel_v_ff_y": "segment_v_ff_y",
    "decel_v_ff_z": "segment_v_ff_z",
    "decel_rho": "segment_rho",
    "decel_dq_max": "segment_dq_max",
    "decel_dqd_max": "segment_dqd_max",
    "decel_gate_joint": "segment_gate_joint",
    "decel_outcome": "segment_outcome",
    "decel_k": "segment_k",
    "decel_n_nodes": "segment_n_nodes",
    "decel_publish_ns": "segment_publish_ns",
    "decel_x0_clamped": "segment_x0_clamped",
    "decel_from_segment": "segment_x0_from_segment",
    "decel_presolved": "segment_presolved",
    "decel_cold_retry": "segment_cold_retry",
    "decel_iterations": "segment_iterations",
    "decel_qp_status": "segment_qp_status",
    "decel_core_reason": "segment_core_reason",
    "decel_solve_us": "segment_solve_us",
    "decel_slack_max": "segment_slack_max",
    "decel_slack_terminal_max": "segment_slack_terminal_max",
    "decel_tau_ratio_max": "segment_tau_ratio_max",
    "decel_kind": "segment_kind",
    "decel_cold_start": "segment_cold_start",
    "decel_solver_retried": "segment_solver_retried",
    "decel_ref_clamped": "segment_ref_clamped",
    "decel_ref_scaled": "segment_ref_scaled",
    "decel_ref_scale": "segment_ref_scale",
    "decel_ref_shortfall": "segment_ref_shortfall",
    "decel_x0_speed": "segment_x0_speed",
    "decel_catch_pos_err": "segment_catch_pos_err",
    "decel_catch_axis_err": "segment_catch_axis_err",
    "decel_catch_gamma": "segment_catch_gamma",
    "decel_catch_v_rel": "segment_catch_v_rel",
    "decel_slack_v": "segment_slack_v",
    "decel_speed_ratio_max": "segment_speed_ratio_max",
    "decel_w_p_fallback": "segment_w_p_fallback",
    "decel_w_delta_scale": "segment_w_delta_scale",
    "decel_source_seq": "segment_source_seq",
}
_NEW_COLUMNS_TO_OLD = {new: old for old, new in RENAMED_COLUMNS.items()}


# ── The tools' own outputs (no compatibility: an old output is regenerated) ──
# ``catching_trials`` writes the segment lane's per-trial metrics as columns of
# ``catching_trials.csv`` (and ``tc_vector`` copies three into ``vec.csv``) and as the
# ``medians`` keys of ``catching_trials_summary.json``, whose lane block moved with them.
RENAMED_TRIALS_COLUMNS: dict[str, str] = {
    "decel_segments_followed": "segment_n_followed",
    "decel_switches": "segment_switches",
    "decel_admitted": "segment_admitted",
    "decel_replaced": "segment_replaced",
    "decel_deferred": "segment_deferred",
    "decel_deferred_max_ticks": "segment_deferred_max_ticks",
    "decel_workspace_refused": "segment_workspace_refused",
    "decel_gate_refused": "segment_gate_refused",
    "decel_aged": "segment_aged",
    "decel_rho_first": "segment_rho_first",
    "decel_rho_replan_max": "segment_rho_replan_max",
    "decel_wait_node0_ms": "segment_wait_node0_ms",
}
RENAMED_TRIALS_SUMMARY_KEYS: dict[str, str] = {"decel_lane": "segment_lane"}


class MixedColumnNamesError(RefusedCatchingInput):
    """A CSV header that holds an old column name together with a new one."""


class OldToolOutputError(RefusedCatchingInput):
    """A tool output (written by another tool of this package) that still has the
    names an older version of that tool wrote — it has to be regenerated."""


def column_renames(columns: Iterable[str], *, source: str = "csv") -> dict[str, str]:
    """``{old: new}`` for the old column names in ``columns`` (empty for a file written
    with the new ones). A header that holds an old name together with a new one — of the
    same column or of another — is refused (:class:`MixedColumnNamesError`): a controller
    writes either all old names or all new ones, so no controller wrote it."""
    cols = list(columns)
    old = sorted({c for c in cols if c in RENAMED_COLUMNS})
    new = sorted({c for c in cols if c in _NEW_COLUMNS_TO_OLD})
    if old and new:
        raise MixedColumnNamesError(
            f"{source}: holds old column names ({', '.join(old)}) together with new ones "
            f"({', '.join(new)})"
        )
    return {c: RENAMED_COLUMNS[c] for c in old}


def normalize_columns(columns: Iterable[str], *, source: str = "csv") -> list[str]:
    """``columns`` with every old column name renamed to the new one
    (:func:`column_renames` refuses a mixed header)."""
    renames = column_renames(columns, source=source)
    return [renames.get(c, c) for c in columns]


def disk_usecols(usecols, renames: Mapping[str, str]):
    """``usecols`` (new names: a list or a ``name -> bool`` callable, or ``None``) as the
    file spells them — for ``pandas.read_csv(usecols=…)`` of a file whose header
    :func:`column_renames` found old names in. A name the file does not have stays as it is."""
    if usecols is None or not renames:
        return usecols
    new_to_old = {new: old for old, new in renames.items()}
    if callable(usecols):
        return lambda name: usecols(renames.get(name, name))
    return [new_to_old.get(c, c) for c in usecols]


def read_csv_normalized(read_csv, path, *, usecols=None, **kwargs):
    """``read_csv(path, usecols=…, **kwargs)`` (``pandas.read_csv``) with the old
    column names of the file renamed to the new ones; ``usecols`` names the new ones.
    A mixed header is refused (:func:`column_renames`)."""
    renames = column_renames(read_csv(path, nrows=0).columns, source=str(path))
    frame = read_csv(path, usecols=disk_usecols(usecols, renames), **kwargs)
    return frame.rename(columns=renames) if renames else frame


def reject_old_output_names(
    names: Iterable[str], old_to_new: Mapping[str, str], *, source: str, tool: str
) -> None:
    """Raise :class:`OldToolOutputError` when ``names`` (the header / keys of a file
    another tool wrote) holds a name that tool no longer writes, naming it and the new
    name: there is no compatibility for an old tool output — regenerate it."""
    found = sorted(n for n in set(names) if n in old_to_new)
    if found:
        lines = ", ".join(f"{n} -> {old_to_new[n]}" for n in found)
        raise OldToolOutputError(
            f"{source}: written by an older {tool} (old name(s): {lines}) — "
            f"regenerate it with the current tool"
        )
