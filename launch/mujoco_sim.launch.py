"""
mujoco_sim.launch.py — MuJoCo 시뮬레이션 런치 파일
=====================================================

기존 실제 로봇 런치(ur_control.launch.py)와 동일한 ROS2 토픽 구조를 사용하므로
custom_controller 노드를 수정 없이 그대로 실행합니다.

사용법:
  # 기본 (뷰어 + 실시간 실행)
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py

  # Headless 모드 (디스플레이 없는 환경)
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py enable_viewer:=false

  # 2배속 시뮬레이션 (데이터 수집 등)
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py sim_speed:=2.0 realtime:=true

  # 최대 속도 (알고리즘 테스트)
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py realtime:=false enable_viewer:=false

  # 외부 Menagerie 모델 사용
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py \\
      model_path:=/path/to/mujoco_menagerie/universal_robots_ur5e/scene.xml

실행되는 노드:
  1. mujoco_simulator_node  — MuJoCo 물리 시뮬레이터 (UR 드라이버 역할 대체)
  2. custom_controller       — 기존 500Hz PD 제어기 (코드 변경 없음)
  3. monitor_data_health.py  — 데이터 헬스 모니터

목표 위치 발행 (별도 터미널):
  ros2 topic pub /target_joint_positions std_msgs/msg/Float64MultiArray \\
    "data: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]"

모니터링:
  ros2 topic hz /joint_states          # 500 Hz 여부 확인
  ros2 topic echo /system/estop_status # E-STOP 상태
  ros2 topic echo /sim/status          # 시뮬레이터 상태
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Launch arguments ─────────────────────────────────────────────────────
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description=(
            'Absolute path to MuJoCo scene.xml. '
            'Empty string → <package>/models/ur5e/scene.xml'
        ),
    )

    enable_viewer_arg = DeclareLaunchArgument(
        'enable_viewer',
        default_value='true',
        description='Open GLFW 3D viewer window (false for headless)',
    )

    realtime_arg = DeclareLaunchArgument(
        'realtime',
        default_value='true',
        description='Synchronise to wall-clock (false = run as fast as possible)',
    )

    sim_speed_arg = DeclareLaunchArgument(
        'sim_speed',
        default_value='1.0',
        description='Real-time speed multiplier (2.0 = 2x faster, requires realtime:=true)',
    )

    control_freq_arg = DeclareLaunchArgument(
        'control_freq',
        default_value='500.0',
        description='Simulation frequency [Hz] — must match ur5e.xml timestep',
    )

    # ── Package paths ─────────────────────────────────────────────────────────
    pkg = FindPackageShare('ur5e_rt_controller')

    sim_config = PathJoinSubstitution([pkg, 'config', 'mujoco_simulator.yaml'])
    ctrl_config = PathJoinSubstitution([pkg, 'config', 'ur5e_rt_controller.yaml'])

    # ── Node 1: MuJoCo Simulator ──────────────────────────────────────────────
    # Publishes  /joint_states, /hand/joint_states
    # Subscribes /forward_position_controller/commands, /hand/command
    mujoco_node = Node(
        package='ur5e_rt_controller',
        executable='mujoco_simulator_node',
        name='mujoco_simulator',
        output='screen',
        emulate_tty=True,
        parameters=[
            sim_config,
            {
                'model_path':    LaunchConfiguration('model_path'),
                'enable_viewer': LaunchConfiguration('enable_viewer'),
                'realtime':      LaunchConfiguration('realtime'),
                'sim_speed':     LaunchConfiguration('sim_speed'),
                'control_freq':  LaunchConfiguration('control_freq'),
            },
        ],
    )

    # ── Node 2: Custom Controller (unchanged) ─────────────────────────────────
    # Subscribes /joint_states, /target_joint_positions, /hand/joint_states
    # Publishes  /forward_position_controller/commands, /system/estop_status
    custom_controller_node = Node(
        package='ur5e_rt_controller',
        executable='custom_controller',
        name='custom_controller',
        output='screen',
        emulate_tty=True,
        parameters=[ctrl_config],
    )

    # ── Node 3: Data Health Monitor ───────────────────────────────────────────
    monitor_node = Node(
        package='ur5e_rt_controller',
        executable='monitor_data_health.py',
        name='data_health_monitor',
        output='screen',
        parameters=[{
            'check_rate':        10.0,
            'timeout_threshold': 0.5,   # relaxed threshold for sim (no RT)
        }],
    )

    return LaunchDescription([
        # Arguments
        model_path_arg,
        enable_viewer_arg,
        realtime_arg,
        sim_speed_arg,
        control_freq_arg,
        # Nodes
        mujoco_node,
        custom_controller_node,
        monitor_node,
    ])
