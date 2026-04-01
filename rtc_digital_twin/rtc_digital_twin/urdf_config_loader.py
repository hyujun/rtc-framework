"""Load robot_description from urdf_pinocchio_bridge YAML config.

Parses the YAML configuration used by urdf_pinocchio_bridge to extract
the URDF path or XML string, resolves xacro if needed, and returns the
processed robot_description XML string.
"""

from __future__ import annotations

import os
import subprocess

import yaml


def load_robot_description(yaml_config_path: str) -> str:
    """Load and return robot_description XML from a pinocchio bridge YAML config.

    The YAML config may specify the URDF source via:
      - ``urdf_path``: file path (absolute, or relative to the YAML file)
      - ``urdf_xml_string``: inline URDF XML string

    If ``urdf_path`` ends with ``.xacro``, it is automatically processed
    via the ``xacro`` command-line tool.

    Args:
        yaml_config_path: Absolute or relative path to the YAML config file.

    Returns:
        Processed URDF XML string suitable for ``robot_description``.

    Raises:
        FileNotFoundError: If the YAML config or resolved URDF file is missing.
        ValueError: If the YAML config contains neither ``urdf_path``
            nor ``urdf_xml_string``.
    """
    yaml_config_path = os.path.abspath(yaml_config_path)
    if not os.path.isfile(yaml_config_path):
        raise FileNotFoundError(
            f'Pinocchio config file not found: {yaml_config_path}')

    with open(yaml_config_path, 'r') as f:
        config = yaml.safe_load(f)

    if not isinstance(config, dict):
        raise ValueError(
            f'Invalid pinocchio config (expected YAML mapping): {yaml_config_path}')

    # Option 1: inline XML string
    urdf_xml = config.get('urdf_xml_string', '')
    if urdf_xml:
        return urdf_xml

    # Option 2: file path
    urdf_path = config.get('urdf_path', '')
    if not urdf_path:
        raise ValueError(
            f'Pinocchio config must contain "urdf_path" or "urdf_xml_string": '
            f'{yaml_config_path}')

    # Resolve relative paths against the YAML file's directory
    if not os.path.isabs(urdf_path):
        yaml_dir = os.path.dirname(yaml_config_path)
        urdf_path = os.path.normpath(os.path.join(yaml_dir, urdf_path))

    if not os.path.isfile(urdf_path):
        raise FileNotFoundError(f'URDF file not found: {urdf_path}')

    if urdf_path.endswith('.xacro'):
        return subprocess.check_output(['xacro', urdf_path], text=True)

    with open(urdf_path, 'r') as f:
        return f.read()
