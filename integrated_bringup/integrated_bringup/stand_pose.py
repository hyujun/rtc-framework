"""The joint controller's stand pose and the time it takes to get there.

Shared by ``run_arms.stand_goal`` (the standing evaluation's ``reentry`` arm)
and the ``sim_recover`` node. Pure: only files of one profile directory
(``config/<profile>``) are read, no ROS.

The stand pose is the joint controller's ``estop.arm_safe_position`` (the sim's
stand keyframe), named by the ``devices.<group>.joint_state_names`` roster of
the profile's ``_base.yaml``.

``trajectory_duration`` repeats the rule of DemoJointController::Compute
(src/controllers/joint/compute.cpp, "Robot arm trajectory"): the quintic starts
at the MEASURED positions and lasts
``max(0.01, max_dist / robot_trajectory_speed, 1.875 * max_dist /
robot_max_traj_velocity)`` seconds, the last term only when the velocity cap is
positive and the speed floored at 1e-6 rad/s (controller.cpp, LoadConfig).
"""

from __future__ import annotations

from pathlib import Path

import yaml

# DemoJointController: floor of the trajectory duration [s] and the quintic's
# rest-to-rest peak-velocity factor (peak = 15/8 * dist / T).
MIN_DURATION_S = 0.01
QUINTIC_PEAK_FACTOR = 1.875
_MIN_SPEED = 1e-6


def _controller_section(profile_dir: Path, joint_controller: str) -> dict:
    path = Path(profile_dir) / "controllers" / f"{joint_controller}.yaml"
    return yaml.safe_load(path.read_text())[joint_controller]


def _find_devices(node, group: str):
    if isinstance(node, dict):
        if isinstance(node.get("devices"), dict) and group in node["devices"]:
            return node["devices"][group]
        for v in node.values():
            found = _find_devices(v, group)
            if found is not None:
                return found
    return None


def stand_pose(
    profile_dir: Path, joint_controller: str, group: str
) -> tuple[list[str], list[float]]:
    """(joint names, stand positions) of ``group`` under ``profile_dir``."""
    profile = Path(profile_dir)
    ctrl = _controller_section(profile, joint_controller)
    pose = [float(x) for x in ctrl["estop"]["arm_safe_position"]]
    device = _find_devices(yaml.safe_load((profile / "_base.yaml").read_text()), group)
    if device is None:
        raise ValueError(f"{profile}/_base.yaml has no devices.{group}")
    names = [str(n) for n in device["joint_state_names"]]
    if len(names) != len(pose):
        raise ValueError(f"{len(pose)} stand-pose values for {len(names)} joints of {group}")
    return names, pose


def trajectory_limits(profile_dir: Path, joint_controller: str) -> tuple[float, float]:
    """(robot_trajectory_speed, robot_max_traj_velocity) [rad/s] of the controller YAML."""
    ctrl = _controller_section(profile_dir, joint_controller)
    return float(ctrl["robot_trajectory_speed"]), float(ctrl["robot_max_traj_velocity"])


def trajectory_duration(max_dist: float, speed: float, max_velocity: float) -> float:
    """Seconds DemoJointController takes for a rest-to-rest move of ``max_dist`` rad."""
    t_speed = max_dist / max(_MIN_SPEED, speed)
    t_vel = QUINTIC_PEAK_FACTOR * max_dist / max_velocity if max_velocity > 0.0 else 0.0
    return max(MIN_DURATION_S, t_speed, t_vel)
