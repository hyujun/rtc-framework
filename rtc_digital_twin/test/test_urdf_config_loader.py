"""Tests for urdf_config_loader — loading robot_description from pinocchio YAML."""

import os
import tempfile
import textwrap

import pytest

from rtc_digital_twin.urdf_config_loader import load_robot_description

SAMPLE_URDF = textwrap.dedent("""\
    <robot name="test">
      <link name="base_link"/>
      <link name="link1"/>
      <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
      </joint>
    </robot>
""")


@pytest.fixture
def tmp_urdf(tmp_path):
    """Write a sample URDF file and return its path."""
    urdf_file = tmp_path / "test_robot.urdf"
    urdf_file.write_text(SAMPLE_URDF)
    return urdf_file


class TestLoadFromUrdfPath:
    def test_absolute_urdf_path(self, tmp_path, tmp_urdf):
        config = tmp_path / "config.yaml"
        config.write_text(f'urdf_path: "{tmp_urdf}"\n')

        result = load_robot_description(str(config))
        assert '<robot name="test">' in result
        assert 'joint1' in result

    def test_relative_urdf_path(self, tmp_path, tmp_urdf):
        config = tmp_path / "config.yaml"
        config.write_text('urdf_path: "test_robot.urdf"\n')

        result = load_robot_description(str(config))
        assert '<robot name="test">' in result

    def test_relative_urdf_path_subdir(self, tmp_path):
        urdf_dir = tmp_path / "urdf"
        urdf_dir.mkdir()
        urdf_file = urdf_dir / "robot.urdf"
        urdf_file.write_text(SAMPLE_URDF)

        config = tmp_path / "config.yaml"
        config.write_text('urdf_path: "urdf/robot.urdf"\n')

        result = load_robot_description(str(config))
        assert '<robot name="test">' in result

    def test_relative_urdf_path_parent_dir(self, tmp_path):
        urdf_file = tmp_path / "robot.urdf"
        urdf_file.write_text(SAMPLE_URDF)

        sub_dir = tmp_path / "configs"
        sub_dir.mkdir()
        config = sub_dir / "config.yaml"
        config.write_text('urdf_path: "../robot.urdf"\n')

        result = load_robot_description(str(config))
        assert '<robot name="test">' in result


class TestLoadFromXmlString:
    def test_inline_xml(self, tmp_path):
        config = tmp_path / "config.yaml"
        indented = textwrap.indent(SAMPLE_URDF, '  ')
        config.write_text(f'urdf_xml_string: |\n{indented}')

        result = load_robot_description(str(config))
        assert '<robot name="test">' in result

    def test_xml_string_takes_priority_over_path(self, tmp_path, tmp_urdf):
        other_urdf = '<robot name="other"><link name="base"/></robot>'
        config = tmp_path / "config.yaml"
        config.write_text(
            f'urdf_xml_string: \'{other_urdf}\'\n'
            f'urdf_path: "{tmp_urdf}"\n'
        )

        result = load_robot_description(str(config))
        assert 'name="other"' in result


class TestErrorCases:
    def test_missing_config_file(self):
        with pytest.raises(FileNotFoundError, match='config file not found'):
            load_robot_description('/nonexistent/config.yaml')

    def test_missing_urdf_file(self, tmp_path):
        config = tmp_path / "config.yaml"
        config.write_text('urdf_path: "nonexistent.urdf"\n')

        with pytest.raises(FileNotFoundError, match='URDF file not found'):
            load_robot_description(str(config))

    def test_no_urdf_source(self, tmp_path):
        config = tmp_path / "config.yaml"
        config.write_text('root_joint_type: "fixed"\n')

        with pytest.raises(ValueError, match='urdf_path.*urdf_xml_string'):
            load_robot_description(str(config))

    def test_invalid_yaml_content(self, tmp_path):
        config = tmp_path / "config.yaml"
        config.write_text('- just a list\n- not a mapping\n')

        with pytest.raises(ValueError, match='expected YAML mapping'):
            load_robot_description(str(config))
