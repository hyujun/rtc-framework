"""
mujoco_sim.launch.py — MuJoCo 시뮬레이션 런치 파일
=====================================================

기존 실제 로봇 런치(ur_control.launch.py)와 동일한 ROS2 토픽 구조를 사용하므로
rt_controller 노드를 수정 없이 그대로 실행합니다.

시뮬레이션 모드:
  free_run  — 최대 속도로 물리 진행 (알고리즘 검증, 궤적 생성)
  sync_step — 제어기 명령 1회 수신 후 1 step 진행 (연산 시간 직접 측정)

사용법:
  # 기본 (YAML 설정 사용)
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py

  # sync_step 모드로 오버라이드
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py sim_mode:=sync_step

  # Headless 모드 (디스플레이 없는 환경)
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py enable_viewer:=false

  # 외부 Menagerie 모델 사용
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py \\
      model_path:=/path/to/mujoco_menagerie/universal_robots_ur5e/scene.xml

  # PD 게인 조정
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py kp:=10.0 kd:=1.0

  # max_rtf 오버라이드 (YAML의 1.0 대신 10.0 사용)
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py max_rtf:=10.0

실행되는 노드:
  1. mujoco_simulator_node  — MuJoCo 물리 시뮬레이터 (UR 드라이버 역할 대체)
  2. rt_controller       — 기존 500Hz PD 제어기 (코드 변경 없음)
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
  import pandas as pd, glob, os
  sessions = sorted(glob.glob(os.path.expanduser('~/ros2_ws/ur5e_ws/logging_data/??????_????')))
  if sessions:
      df = pd.read_csv(os.path.join(sessions[-1], 'controller', 'timing_log.csv'))
      print(df['t_compute_us'].describe())
      print(f'P95: {df[\"t_compute_us\"].quantile(0.95):.1f} us')
      print(f'P99: {df[\"t_compute_us\"].quantile(0.99):.1f} us')
      print(f'Over 2ms: {(df[\"t_compute_us\"] > 2000).mean()*100:.2f}%')
  "
"""

import os
import re
import shutil
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def _resolve_logging_root():
    """colcon workspace 기반 logging_data 루트 경로 결정."""
    try:
        share_dir = get_package_share_directory('ur5e_rt_controller')
        ws_dir = os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.dirname(share_dir))))
        return os.path.join(ws_dir, 'logging_data')
    except Exception:
        return os.path.expanduser('~/ros2_ws/ur5e_ws/logging_data')


def _cleanup_old_sessions(logging_root, max_sessions):
    """YYMMDD_HHMM 패턴 세션 폴더를 max_sessions 개수 이하로 유지."""
    if not os.path.isdir(logging_root):
        return
    pattern = re.compile(r'^\d{6}_\d{4}$')
    dirs = sorted([
        d for d in os.listdir(logging_root)
        if os.path.isdir(os.path.join(logging_root, d)) and pattern.match(d)
    ])
    while len(dirs) > max_sessions:
        oldest = os.path.join(logging_root, dirs.pop(0))
        shutil.rmtree(oldest, ignore_errors=True)


def launch_setup(context, *args, **kwargs):
    """Setup function executed with launch context for conditional parameter loading."""

    # ── 세션 디렉토리 생성 (YYMMDD_HHMM) ─────────────────────────────────────
    logging_root = _resolve_logging_root()
    session_ts = datetime.now().strftime('%y%m%d_%H%M')
    session_dir = os.path.join(logging_root, session_ts)
    for sub in ('controller', 'monitor', 'hand', 'sim', 'plots', 'motions'):
        os.makedirs(os.path.join(session_dir, sub), exist_ok=True)

    max_sessions = int(
        LaunchConfiguration('max_log_sessions').perform(context) or '10')
    _cleanup_old_sessions(logging_root, max_sessions)

    # ── Package paths ─────────────────────────────────────────────────────────
    pkg_sim = FindPackageShare('ur5e_mujoco_sim')
    pkg_ctrl = FindPackageShare('ur5e_rt_controller')

    sim_config = PathJoinSubstitution(
        [pkg_sim,  'config', 'mujoco_simulator.yaml'])
    ctrl_config = PathJoinSubstitution(
        [pkg_ctrl, 'config', 'ur5e_rt_controller.yaml'])

    # ── Build simulator parameters (YAML first, then conditional overrides) ───
    sim_params = [sim_config]
    sim_overrides = {}

    # Check each launch argument - only add to overrides if explicitly provided
    model_path = LaunchConfiguration('model_path').perform(context)
    if model_path != '':
        sim_overrides['model_path'] = model_path

    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    if sim_mode != '':
        sim_overrides['sim_mode'] = sim_mode

    enable_viewer = LaunchConfiguration('enable_viewer').perform(context)
    if enable_viewer != '':
        # Convert string to bool
        sim_overrides['enable_viewer'] = enable_viewer.lower() in (
            'true', '1', 'yes')

    publish_decimation = LaunchConfiguration(
        'publish_decimation').perform(context)
    if publish_decimation != '':
        sim_overrides['publish_decimation'] = int(publish_decimation)

    sync_timeout_ms = LaunchConfiguration('sync_timeout_ms').perform(context)
    if sync_timeout_ms != '':
        sim_overrides['sync_timeout_ms'] = float(sync_timeout_ms)

    max_rtf = LaunchConfiguration('max_rtf').perform(context)
    if max_rtf != '':
        sim_overrides['max_rtf'] = float(max_rtf)

    use_yaml_servo_gains = LaunchConfiguration(
        'use_yaml_servo_gains').perform(context)
    if use_yaml_servo_gains != '':
        sim_overrides['use_yaml_servo_gains'] = (
            use_yaml_servo_gains.lower() in ('true', '1', 'yes'))

    # Add overrides only if any were provided
    if sim_overrides:
        sim_params.append(sim_overrides)

    # ── Build controller parameters (YAML + overrides + launch args) ──────────
    ctrl_params = [ctrl_config, sim_config]
    ctrl_overrides = {}

    kp = LaunchConfiguration('kp').perform(context)
    if kp != '':
        ctrl_overrides['kp'] = float(kp)

    kd = LaunchConfiguration('kd').perform(context)
    if kd != '':
        ctrl_overrides['kd'] = float(kd)

    # 세션 디렉토리를 log_dir로 전달 (rt_controller가 session_dir 내에서 로깅)
    ctrl_overrides['log_dir'] = session_dir

    if ctrl_overrides:
        ctrl_params.append(ctrl_overrides)

    # ── 환경변수: 모든 노드에 세션 디렉토리 전파 ────────────────────────────────
    set_session_dir = SetEnvironmentVariable(
        name='UR5E_SESSION_DIR',
        value=session_dir
    )

    # ── Node 1: MuJoCo Simulator ──────────────────────────────────────────────
    mujoco_node = Node(
        package='ur5e_mujoco_sim',
        executable='mujoco_simulator_node',
        name='mujoco_simulator',
        output='screen',
        emulate_tty=True,
        parameters=sim_params,
    )

    # ── Node 2: Custom Controller ─────────────────────────────────────────────
    rt_controller_node = Node(
        package='ur5e_rt_controller',
        executable='rt_controller',
        name='rt_controller',
        output='screen',
        emulate_tty=True,
        parameters=ctrl_params,
    )

    # ── Node 3: Data Health Monitor ───────────────────────────────────────────
    monitor_node = Node(
        package='ur5e_tools',
        executable='monitor_data_health',
        name='data_health_monitor',
        output='screen',
        parameters=[{
            'check_rate':        10.0,
            'timeout_threshold': 1.0,
        }],
    )

    return [set_session_dir, mujoco_node, rt_controller_node, monitor_node]


def generate_launch_description():
    # ── Launch arguments with empty defaults (YAML values take precedence) ───
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description=(
            'Override model_path from YAML. '
            'Empty → use YAML value (ur5e_description/scene.xml). '
            'Absolute path → use specified MuJoCo scene.xml'
        ),
    )

    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='',
        description=(
            'Override sim_mode from YAML. '
            'Empty → use YAML value (free_run). '
            'Options: "free_run" (max speed) or "sync_step" (1:1 sync)'
        ),
    )

    enable_viewer_arg = DeclareLaunchArgument(
        'enable_viewer',
        default_value='',
        description=(
            'Override enable_viewer from YAML. '
            'Empty → use YAML value (true). '
            'Set to "false" for headless mode'
        ),
    )

    publish_decimation_arg = DeclareLaunchArgument(
        'publish_decimation',
        default_value='',
        description=(
            'Override publish_decimation from YAML. '
            'Empty → use YAML value (1). '
            'free_run only: publish /joint_states every N physics steps'
        ),
    )

    sync_timeout_ms_arg = DeclareLaunchArgument(
        'sync_timeout_ms',
        default_value='',
        description=(
            'Override sync_timeout_ms from YAML. '
            'Empty → use YAML value (50.0). '
            'sync_step only: command wait timeout in milliseconds'
        ),
    )

    max_rtf_arg = DeclareLaunchArgument(
        'max_rtf',
        default_value='',
        description=(
            'Override max_rtf from YAML. '
            'Empty → use YAML value (1.0). '
            'Maximum Real-Time Factor (0.0 = unlimited). '
            'Examples: 1.0 for real-time, 10.0 for 10x speed'
        ),
    )

    kp_arg = DeclareLaunchArgument(
        'kp',
        default_value='',
        description=(
            'Override kp from YAML. '
            'Empty → use YAML value. '
            'PD controller proportional gain'
        ),
    )

    kd_arg = DeclareLaunchArgument(
        'kd',
        default_value='',
        description=(
            'Override kd from YAML. '
            'Empty → use YAML value. '
            'PD controller derivative gain'
        ),
    )

    use_yaml_servo_gains_arg = DeclareLaunchArgument(
        'use_yaml_servo_gains',
        default_value='',
        description=(
            'Override use_yaml_servo_gains from YAML. '
            'Empty → use YAML value (false). '
            'true: servo_kp/kd gains from YAML, false: XML gainprm/biasprm'
        ),
    )

    max_log_sessions_arg = DeclareLaunchArgument(
        'max_log_sessions',
        default_value='10',
        description='최대 보관 세션 폴더 수 (YYMMDD_HHMM)',
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
        use_yaml_servo_gains_arg,
        max_log_sessions_arg,
        # Nodes (via OpaqueFunction for conditional parameter loading)
        OpaqueFunction(function=launch_setup),
    ])
