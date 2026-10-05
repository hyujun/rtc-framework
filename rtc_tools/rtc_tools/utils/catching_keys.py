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
"""

from __future__ import annotations

from collections.abc import Mapping

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


class RenamedCatchingKeyError(ValueError):
    """A catching profile that still writes a key that moved."""


class MixedMirrorNamesError(ValueError):
    """A recorded mirror that holds an old name together with its new name."""


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
            "rename them (#711); this tool would otherwise read the default"
        )


def reject_renamed_keys_in_config(config: object, *, source: str = "controller config") -> None:
    """:func:`reject_renamed_keys` for a composed controller config
    (``{<config_key>: {catching: …}}``): every top-level value that holds a
    ``catching`` map is judged; a bare ``{catching: …}`` is judged too."""
    if not isinstance(config, Mapping):
        return
    if isinstance(config.get("catching"), Mapping):
        reject_renamed_keys(config["catching"], source=source)
    for key, node in config.items():
        if isinstance(node, Mapping) and isinstance(node.get("catching"), Mapping):
            reject_renamed_keys(node["catching"], source=f"{source} [{key}]")


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
