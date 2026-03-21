#!/usr/bin/env python3
"""Convert URDF or XACRO files to MuJoCo MJCF XML.

Uses the xacro package to process .xacro files and mujoco's built-in
URDF compiler to produce the final MJCF XML.

Usage:
  ros2 run rtc_tools urdf_to_mjcf --input /path/to/robot.urdf
  ros2 run rtc_tools urdf_to_mjcf --input /path/to/robot.urdf.xacro
  ros2 run rtc_tools urdf_to_mjcf --input /path/to/robot.xacro --output /path/to/output.xml
  ros2 run rtc_tools urdf_to_mjcf --input /path/to/robot.xacro --xacro-args name:=ur5e safety_limits:=true
"""

import argparse
import re
import os
import sys
import tempfile
from pathlib import Path
from typing import Optional

import mujoco


def process_xacro(xacro_path: Path, xacro_args: Optional[list[str]] = None) -> str:
    """Process a xacro file and return URDF XML string."""
    try:
        import xacro
    except ImportError:
        print("Error: 'xacro' package is required for .xacro files.", file=sys.stderr)
        print("Install with: pip install xacro  or  apt install ros-${ROS_DISTRO}-xacro", file=sys.stderr)
        sys.exit(1)

    mappings = {}
    if xacro_args:
        for arg in xacro_args:
            if ":=" in arg:
                key, val = arg.split(":=", 1)
                mappings[key] = val
            else:
                print(f"Warning: ignoring invalid xacro arg '{arg}' (expected key:=value)", file=sys.stderr)

    doc = xacro.process_file(str(xacro_path), mappings=mappings)
    return doc.toprettyxml(indent="  ")


def is_xacro_file(path: Path) -> bool:
    """Check if the file is a xacro file by extension."""
    return ".xacro" in [s.lower() for s in path.suffixes]


def resolve_package_uris(urdf_xml: str) -> str:
    """Replace package:// URIs with absolute paths using ament index."""
    pkg_cache: dict[str, str] = {}

    def _resolve_pkg(pkg_name: str) -> Optional[str]:
        if pkg_name in pkg_cache:
            return pkg_cache[pkg_name]
        resolved = None
        try:
            from ament_index_python.packages import get_package_share_directory
            resolved = get_package_share_directory(pkg_name)
        except Exception:
            # Fallback: search in workspace src directory
            ws_src = Path(__file__).resolve().parents[3]
            candidate = ws_src / pkg_name
            if candidate.is_dir():
                resolved = str(candidate)
        pkg_cache[pkg_name] = resolved
        return resolved

    def _replace(match: re.Match) -> str:
        pkg_name = match.group(1)
        rel_path = match.group(2)
        pkg_dir = _resolve_pkg(pkg_name)
        if pkg_dir:
            return str(Path(pkg_dir) / rel_path)
        return match.group(0)

    return re.sub(r"package://([^/]+)/([^\s\"'<>]+)", _replace, urdf_xml)


def convert_urdf_to_mjcf(
    input_path: Path,
    output_path: Optional[Path] = None,
    xacro_args: Optional[list[str]] = None,
    meshdir: Optional[Path] = None,
) -> Path:
    """Convert a URDF/XACRO file to MuJoCo MJCF XML.

    Args:
        input_path: Path to .urdf or .xacro file.
        output_path: Output MJCF path. Defaults to <input_stem>.xml in same directory.
        xacro_args: Extra xacro arguments (key:=value).
        meshdir: Directory containing mesh files. Defaults to input file's directory.

    Returns:
        Path to the generated MJCF XML file.
    """
    input_path = input_path.resolve()
    if not input_path.exists():
        raise FileNotFoundError(f"Input file not found: {input_path}")

    if meshdir is None:
        meshdir = input_path.parent

    # Determine output path
    if output_path is None:
        stem = input_path.stem
        # Handle double extensions like robot.urdf.xacro
        if stem.endswith(".urdf"):
            stem = stem[:-5]
        output_path = input_path.parent / f"{stem}_mjcf.xml"

    # Step 1: Get URDF XML (process xacro if needed)
    if is_xacro_file(input_path):
        print(f"Processing xacro: {input_path}")
        urdf_xml = process_xacro(input_path, xacro_args)
    else:
        urdf_xml = input_path.read_text()

    # Step 2: Resolve package:// URIs to absolute paths
    urdf_xml = resolve_package_uris(urdf_xml)

    # Write to a temp file for mujoco to load
    tmp_dir = tempfile.mkdtemp(prefix="urdf_to_mjcf_")
    tmp_urdf = Path(tmp_dir) / "robot.urdf"
    tmp_urdf.write_text(urdf_xml)
    urdf_path = tmp_urdf

    # Step 3: Load URDF into MuJoCo
    print(f"Loading URDF into MuJoCo...")
    original_dir = os.getcwd()
    try:
        # Change to mesh directory so MuJoCo can find mesh files
        os.chdir(meshdir)
        model = mujoco.MjModel.from_xml_path(str(urdf_path))
    except Exception as e:
        print(f"Error: MuJoCo failed to compile URDF: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        os.chdir(original_dir)

    # Step 4: Save as MJCF XML
    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    mujoco.mj_saveLastXML(str(output_path), model)

    print(f"MJCF saved: {output_path}")
    print(f"  Bodies:    {model.nbody}")
    print(f"  Joints:    {model.njnt}")
    print(f"  DOF:       {model.nv}")
    print(f"  Geoms:     {model.ngeom}")
    print(f"  Meshes:    {model.nmesh}")
    print(f"  Actuators: {model.nu}")

    return output_path


def main():
    parser = argparse.ArgumentParser(
        description="Convert URDF/XACRO to MuJoCo MJCF XML.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
  ros2 run rtc_tools urdf_to_mjcf --input robot.urdf
  ros2 run rtc_tools urdf_to_mjcf --input robot.urdf.xacro --output robot_mj.xml
  ros2 run rtc_tools urdf_to_mjcf --input robot.xacro --xacro-args name:=ur5e
  ros2 run rtc_tools urdf_to_mjcf --input robot.urdf --meshdir /path/to/meshes
""",
    )
    parser.add_argument(
        "--input", "-i", type=Path, required=True,
        help="Path to input URDF or XACRO file.",
    )
    parser.add_argument(
        "--output", "-o", type=Path, default=None,
        help="Path to output MJCF XML file (default: <input_stem>_mjcf.xml).",
    )
    parser.add_argument(
        "--meshdir", "-m", type=Path, default=None,
        help="Directory containing mesh files (default: input file's directory).",
    )
    parser.add_argument(
        "--xacro-args", nargs="*", default=None,
        help="Extra xacro arguments in key:=value format.",
    )

    args = parser.parse_args()
    convert_urdf_to_mjcf(args.input, args.output, args.xacro_args, args.meshdir)


if __name__ == "__main__":
    main()
