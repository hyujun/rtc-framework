"""Do a controller model (URDF) and a simulated plant (MJCF) describe the same body?

Three numbers, at the models' zero configuration: the difference in total
mass, in whole-body centre of mass and in composite inertia about that centre.
They answer one question — is a body present in one model and absent from the
other — and a run's validity is judged on them: a controller that carries a
hand its plant does not have is not being tested for robustness, its model is
simply wrong by that hand. Which link differs is ``compare_mjcf_urdf``'s job;
this is the cheap check a tool can make on every run.

The centre of mass and the inertia are expressed in the root link's frame, so a
floating base placed anywhere in the plant's keyframe does not enter. The MJCF's
first body is taken to be the URDF's root link.

Needs pinocchio and mujoco (and xacro for a ``.xacro`` source).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


def urdf_xml(path: Path) -> str:
    """The URDF text; a ``.xacro`` is expanded first."""
    if path.suffix != ".xacro":
        return path.read_text()
    import xacro

    return xacro.process_file(str(path)).toxml()


def control_inertia(xml: str, floating: bool):
    """Whole-model composite inertia in the root link's frame (joints at zero)."""
    import pinocchio as pin

    model = (
        pin.buildModelFromXML(xml, pin.JointModelFreeFlyer())
        if floating
        else pin.buildModelFromXML(xml)
    )
    data = model.createData()
    q = pin.neutral(model)
    pin.ccrba(model, data, q, np.zeros(model.nv))
    com = pin.centerOfMass(model, data, q)
    return pin.Inertia(pin.computeTotalMass(model), com, data.Ig.inertia)


def plant_inertia(mjcf: Path):
    """Composite inertia of every body of the plant, in its first body's frame (qpos0)."""
    import mujoco
    import pinocchio as pin

    model = mujoco.MjModel.from_xml_path(str(mjcf))
    data = mujoco.MjData(model)
    data.qpos[:] = model.qpos0
    mujoco.mj_forward(model, data)
    if model.nbody < 2:
        raise ValueError(f"{mjcf}: no body")
    root_rot = data.xmat[1].reshape(3, 3)
    total = pin.Inertia.Zero()
    for i in range(1, model.nbody):
        if model.body_mass[i] <= 0.0:
            continue
        placement = pin.SE3(
            root_rot.T @ data.ximat[i].reshape(3, 3), root_rot.T @ (data.xipos[i] - data.xpos[1])
        )
        total += placement.act(
            pin.Inertia(model.body_mass[i], np.zeros(3), np.diag(model.body_inertia[i]))
        )
    return total


def model_match(urdf: Path, mjcf: Path, floating: bool = True) -> dict:
    """Plant − control: ``mass_kg`` (signed), ``com_m`` (distance), ``inertia_kgm2`` (max |Δ|)."""
    control = control_inertia(urdf_xml(urdf), floating)
    plant = plant_inertia(mjcf)
    return {
        "control_mass_kg": float(control.mass),
        "plant_mass_kg": float(plant.mass),
        "mass_kg": float(plant.mass - control.mass),
        "com_m": float(np.linalg.norm(plant.lever - control.lever)),
        "inertia_kgm2": float(np.abs(plant.inertia - control.inertia).max()),
    }


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("urdf", type=Path, help="the controller model (.urdf or .urdf.xacro)")
    p.add_argument("mjcf", type=Path, help="the simulated plant")
    p.add_argument("--fixed-base", action="store_true", help="the URDF root is fixed")
    args = p.parse_args(argv)
    print(json.dumps(model_match(args.urdf, args.mjcf, not args.fixed_base), indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
