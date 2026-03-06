"""
mujoco_sim.launch.py — MuJoCo 시뮬레이션 런치 파일
=====================================================

기존 실제 로봇 런치(ur_control.launch.py)와 동일한 ROS2 토픽 구조를 사용하므로
custom_controller 노드를 수정 없이 그대로 실행합니다.

시뮬레이션 모드:
  free_run  — 최대 속도로 물리 진행 (알고리즘 검증, 궤적 생성)
  sync_step — 제어기 명령 1회 수신 후 1 step 진행 (연산 시간 직접 측정)

사용법:
  # 기본 (free_run, 뷰어 활성화)
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py

  # sync_step 모드 (제어기 Compute() 시간 측정)
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py sim_mode:=sync_step

  # Headless 모드 (디스플레이 없는 환경)
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py enable_viewer:=false

  # 외부 Menagerie 모델 사용
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py \\
      model_path:=/path/to/mujoco_menagerie/universal_robots_ur5e/scene.xml

  # PD 게인 조정
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py kp:=10.0 kd:=1.0

실행되는 노드:
  1. mujoco_simulator_node  — MuJoCo 물리 시뮬레이터 (UR 드라이버 역할 대체)
  2. custom_controller       — 기존 500Hz PD 제어기 (코드 변경 없음)
  3. monitor_data_health.py  — 데이터 헬스 모니터

목표 위치 발행 (별도 터미널):
  ros2 topic pub /target_joint_positions std_msgs/msg/Float64MultiArray \\
    "data: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]"

모니터링:
  ros2 topic hz /joint_states            # 게시 주파수 확인
  ros2 topic echo /system/estop_status   # E-STOP 상태
  ros2 topic echo /sim/status            # 시뮬레이터 상태 (steps, sim_time)

연산 시간 로그 분석 (sync_step 실행 후):
  python3 -c "
  import pandas as pd
  df = pd.read_csv('~/ur_ws/logging_data/ur5e_control_log_YYMMDD_HHMM.csv')
  print(df['compute_time_us'].describe())
  print(f'P95: {df[\"compute_time_us\"].quantile(0.95):.1f} us')
  print(f'P99: {df[\"compute_time_us\"].quantile(0.99):.1f} us')
  print(f'Over 2ms: {(df[\"compute_time_us\"] > 2000).mean()*100:.2f}%')
  "
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Log directory (colcon workspace logging_data/) ────────────────────────
    log_dir = os.path.expanduser('~/ur_ws/logging_data')

    # ── Launch arguments ─────────────────────────────────────────────────────
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description=(
            'Absolute path to MuJoCo scene.xml. '
            'Empty string → <package>/models/ur5e/scene.xml'
        ),
    )

    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='free_run',
        description=(
            'Simulation mode: '
            '"free_run" (max speed, best for validation) or '
            '"sync_step" (1:1 sync, measures Compute() time)'
        ),
    )

    enable_viewer_arg = DeclareLaunchArgument(
        'enable_viewer',
        default_value='true',
        description='Open GLFW 3D viewer window (false for headless)',
    )

    publish_decimation_arg = DeclareLaunchArgument(
        'publish_decimation',
        default_value='1',
        description=(
            'free_run only: publish /joint_states every N physics steps. '
            'Increase to reduce DDS load in fast simulations.'
        ),
    )

    sync_timeout_ms_arg = DeclareLaunchArgument(
        'sync_timeout_ms',
        default_value='50.0',
        description='sync_step only: command wait timeout in milliseconds',
    )

    max_rtf_arg = DeclareLaunchArgument(
        'max_rtf',
        default_value='0.0',
        description=(
            'Maximum Real-Time Factor (0.0 = unlimited). '
            'The sim loop sleeps after each mj_step() to keep RTF <= max_rtf. '
            'Example: max_rtf:=1.0 for real-time, max_rtf:=10.0 for 10x.'
        ),
    )

    kp_arg = DeclareLaunchArgument(
        'kp',
        default_value='5.0',
        description='PD controller proportional gain',
    )

    kd_arg = DeclareLaunchArgument(
        'kd',
        default_value='0.5',
        description='PD controller derivative gain',
    )

    # ── Package paths ─────────────────────────────────────────────────────────
    pkg_sim  = FindPackageShare('ur5e_mujoco_sim')
    pkg_ctrl = FindPackageShare('ur5e_rt_controller')

    sim_config  = PathJoinSubstitution([pkg_sim,  'config', 'mujoco_simulator.yaml'])
    ctrl_config = PathJoinSubstitution([pkg_ctrl, 'config', 'ur5e_rt_controller.yaml'])

    # ── Node 1: MuJoCo Simulator ──────────────────────────────────────────────
    # Publishes  /joint_states, /hand/joint_states, /sim/status
    # Subscribes /forward_position_controller/commands, /hand/command
    mujoco_node = Node(
        package='ur5e_mujoco_sim',
        executable='mujoco_simulator_node',
        name='mujoco_simulator',
        output='screen',
        emulate_tty=True,
        parameters=[
            sim_config,
            {
                'model_path':         LaunchConfiguration('model_path'),
                'sim_mode':           LaunchConfiguration('sim_mode'),
                'enable_viewer':      LaunchConfiguration('enable_viewer'),
                'publish_decimation': LaunchConfiguration('publish_decimation'),
                'sync_timeout_ms':    LaunchConfiguration('sync_timeout_ms'),
                'max_rtf':            LaunchConfiguration('max_rtf'),
            },
        ],
    )

    # ── Node 2: Custom Controller (unchanged) ─────────────────────────────────
    # Subscribes /joint_states, /target_joint_positions, /hand/joint_states
    # Publishes  /forward_position_controller/commands, /system/estop_status
    #
    # Parameter load order (later entries override earlier ones):
    #   1. ur5e_rt_controller.yaml — base gains / logging settings
    #   2. mujoco_simulator.yaml   — sim-specific overrides (enable_estop: false,
    #                                extended timeouts)
    #   3. Inline dict             — kp / kd from launch arguments
    custom_controller_node = Node(
        package='ur5e_rt_controller',
        executable='custom_controller',
        name='custom_controller',
        output='screen',
        emulate_tty=True,
        parameters=[
            ctrl_config,
            sim_config,   # provides custom_controller.ros__parameters overrides
            {
                'kp': LaunchConfiguration('kp'),
                'kd': LaunchConfiguration('kd'),
                'log_dir': log_dir,
            },
        ],
    )

    # ── Node 3: Data Health Monitor ───────────────────────────────────────────
    monitor_node = Node(
        package='ur5e_tools',
        executable='monitor_data_health.py',
        name='data_health_monitor',
        output='screen',
        parameters=[{
            'check_rate':        10.0,
            'timeout_threshold': 1.0,   # relaxed for sim (free_run may be bursty)
        }],
    )

    return LaunchDescription([
        # Arguments
        model_path_arg,
        sim_mode_arg,
        enable_viewer_arg,
        publish_decimation_arg,
        sync_timeout_ms_arg,
        max_rtf_arg,
        kp_arg,
        kd_arg,
        # Nodes
        mujoco_node,
        custom_controller_node,
        monitor_node,
    ])
