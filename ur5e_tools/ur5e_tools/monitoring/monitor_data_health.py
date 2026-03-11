#!/usr/bin/env python3
"""
monitor_data_health.py - v3 (Per-Controller Statistics)

실시간 데이터 헬스 모니터링 + 컨트롤러 종류별 통계 JSON 저장.

저장 구조:
  logging_data/stats/
    <controller_name>/
      health_stats_YYYYMMDD_HHMMSS.json
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool, String
import json
from datetime import datetime
from pathlib import Path


class DataHealthMonitor(Node):
    def __init__(self):
        super().__init__('data_health_monitor')

        import os
        from ament_index_python.packages import get_package_share_directory

        try:
            share_dir = get_package_share_directory('ur5e_rt_controller')
            ws_dir = os.path.dirname(os.path.dirname(
                os.path.dirname(os.path.dirname(share_dir))))
            default_stats_dir = os.path.join(ws_dir, 'logging_data', 'stats')
        except Exception:
            default_stats_dir = os.path.expanduser(
                '~/ros2_ws/ur5e_ws/logging_data/stats')

        # Parameters
        self.declare_parameter('check_rate', 10.0)
        self.declare_parameter('timeout_threshold', 0.2)
        self.declare_parameter('stats_output_dir', default_stats_dir)
        self.declare_parameter('enable_stats', True)

        self.check_rate = self.get_parameter('check_rate').value
        self.timeout_threshold = self.get_parameter('timeout_threshold').value
        self.stats_dir = Path(self.get_parameter('stats_output_dir').value)
        self.enable_stats = self.get_parameter('enable_stats').value

        # 현재 활성 컨트롤러 이름 (~/active_controller_name 수신 전 기본값)
        self.current_controller = 'unknown'

        # 컨트롤러별 누적 통계: {controller_name: stats_dict}
        self.per_controller_stats = {}

        # 타임스탬프 추적
        self.last_robot_time = None
        self.last_hand_time = None
        self.last_command_time = None

        self.robot_timeout_count = 0
        self.hand_timeout_count = 0

        # 컨트롤러 이름 구독 — transient_local(latched) QoS
        latch_qos = QoSProfile(depth=1,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ctrl_name_sub = self.create_subscription(
            String, '/rt_controller/active_controller_name',
            self._controller_name_callback, latch_qos)

        # 데이터 구독
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        self.hand_state_sub = self.create_subscription(
            Float64MultiArray, '/hand/joint_states', self.hand_state_callback, 10)

        self.command_sub = self.create_subscription(
            Float64MultiArray, '/forward_position_controller/commands',
            self.command_callback, 10)

        self.estop_sub = self.create_subscription(
            Bool, '/system/estop_status', self.estop_callback, 10)

        # 헬스 체크 타이머
        self.timer = self.create_timer(
            1.0 / self.check_rate, self.check_health)

        self.get_logger().info('Data Health Monitor v3 started')
        self.get_logger().info(f'Check rate: {self.check_rate} Hz')
        self.get_logger().info(
            f'Timeout threshold: {self.timeout_threshold} s')
        if self.enable_stats:
            self.get_logger().info(f'Statistics output: {self.stats_dir}')

    # ── 컨트롤러별 stats 관리 ────────────────────────────────────────────────

    def _make_empty_stats(self):
        """컨트롤러 하나의 빈 통계 블록을 반환."""
        return {
            'first_active': datetime.now().isoformat(),
            'last_active': None,
            'total_active_s': 0.0,
            '_session_start': datetime.now().isoformat(),   # 내부용 (저장 제외)
            'robot': {
                'total_packets': 0,
                'lost_packets': 0,
                'avg_rate': 0.0,
                'last_update': None,
            },
            'hand': {
                'total_packets': 0,
                'lost_packets': 0,
                'avg_rate': 0.0,
                'last_update': None,
            },
            'commands': {
                'total_packets': 0,
                'avg_rate': 0.0,
                'last_update': None,
            },
            'estop': {
                'total_triggers': 0,
                'current_status': False,
                'last_trigger': None,
            },
        }

    def _current_stats(self):
        """현재 컨트롤러의 stats dict 반환 (없으면 생성)."""
        if self.current_controller not in self.per_controller_stats:
            self.per_controller_stats[self.current_controller] = \
                self._make_empty_stats()
        return self.per_controller_stats[self.current_controller]

    def _controller_name_callback(self, msg):
        new_name = msg.data
        if not new_name:
            return
        if new_name == self.current_controller:
            return

        # 이전 컨트롤러 세션 종료 기록
        if self.current_controller in self.per_controller_stats:
            prev = self.per_controller_stats[self.current_controller]
            now_str = datetime.now().isoformat()
            prev['last_active'] = now_str
            # 이번 세션 활성 시간 누산
            session_start = datetime.fromisoformat(prev['_session_start'])
            elapsed = (datetime.now() - session_start).total_seconds()
            prev['total_active_s'] = prev.get('total_active_s', 0.0) + elapsed

        self.current_controller = new_name

        # 신규 컨트롤러 진입 시 세션 시작 시간 갱신
        stats = self._current_stats()
        stats['_session_start'] = datetime.now().isoformat()

        self.get_logger().info(f'Controller changed → {new_name}')

    # ── 데이터 콜백 ─────────────────────────────────────────────────────────

    def joint_state_callback(self, msg):
        self.last_robot_time = self.get_clock().now()
        s = self._current_stats()
        s['robot']['total_packets'] += 1
        s['robot']['last_update'] = datetime.now().isoformat()

    def hand_state_callback(self, msg):
        self.last_hand_time = self.get_clock().now()
        s = self._current_stats()
        s['hand']['total_packets'] += 1
        s['hand']['last_update'] = datetime.now().isoformat()

    def command_callback(self, msg):
        self.last_command_time = self.get_clock().now()
        s = self._current_stats()
        s['commands']['total_packets'] += 1
        s['commands']['last_update'] = datetime.now().isoformat()

    def estop_callback(self, msg):
        s = self._current_stats()
        if msg.data and not s['estop']['current_status']:
            s['estop']['total_triggers'] += 1
            s['estop']['last_trigger'] = datetime.now().isoformat()
            self.get_logger().error('E-STOP TRIGGERED!')
        s['estop']['current_status'] = msg.data

    # ── 헬스 체크 ───────────────────────────────────────────────────────────

    def check_health(self):
        now = self.get_clock().now()
        s = self._current_stats()

        if self.last_robot_time is not None:
            robot_age = (now - self.last_robot_time).nanoseconds / 1e9
            if robot_age > self.timeout_threshold:
                self.robot_timeout_count += 1
                s['robot']['lost_packets'] += 1
                self.get_logger().warn(
                    f'Robot data timeout: {robot_age:.3f}s '
                    f'(count: {self.robot_timeout_count})')

        if self.last_hand_time is not None:
            hand_age = (now - self.last_hand_time).nanoseconds / 1e9
            if hand_age > self.timeout_threshold:
                self.hand_timeout_count += 1
                s['hand']['lost_packets'] += 1
                self.get_logger().warn(
                    f'Hand data timeout: {hand_age:.3f}s '
                    f'(count: {self.hand_timeout_count})')

        robot_packets = s['robot']['total_packets']
        if robot_packets % 100 == 0 and robot_packets > 0:
            self.log_status()

    def log_status(self):
        s = self._current_stats()
        robot_packets = s['robot']['total_packets']
        robot_lost = s['robot']['lost_packets']
        hand_packets = s['hand']['total_packets']
        hand_lost = s['hand']['lost_packets']

        robot_loss_rate = (robot_lost / robot_packets * 100) if robot_packets > 0 else 0
        hand_loss_rate = (hand_lost / hand_packets * 100) if hand_packets > 0 else 0

        self.get_logger().info(
            f'[{self.current_controller}] === Data Health Status ===')
        self.get_logger().info(
            f'  Robot:    {robot_packets} packets, '
            f'{robot_lost} lost ({robot_loss_rate:.2f}%)')
        self.get_logger().info(
            f'  Hand:     {hand_packets} packets, '
            f'{hand_lost} lost ({hand_loss_rate:.2f}%)')
        self.get_logger().info(
            f'  Commands: {s["commands"]["total_packets"]} sent')
        self.get_logger().info(
            f'  E-STOP:   {"ACTIVE" if s["estop"]["current_status"] else "CLEAR"} '
            f'(triggered {s["estop"]["total_triggers"]} times)')

    # ── 통계 저장 ────────────────────────────────────────────────────────────

    def save_statistics(self):
        if not self.enable_stats or not self.per_controller_stats:
            return

        now = datetime.now()

        # 현재 세션 종료 처리 (last_active 기록 + 누산)
        if self.current_controller in self.per_controller_stats:
            cur = self.per_controller_stats[self.current_controller]
            cur['last_active'] = now.isoformat()
            session_start = datetime.fromisoformat(cur['_session_start'])
            elapsed = (now - session_start).total_seconds()
            cur['total_active_s'] = cur.get('total_active_s', 0.0) + elapsed

        filename = f"health_stats_{now.strftime('%Y%m%d_%H%M%S')}.json"

        for ctrl_name, stats in self.per_controller_stats.items():
            # avg_rate 계산
            total_s = stats.get('total_active_s', 0.0)
            if total_s > 0:
                stats['robot']['avg_rate'] = \
                    stats['robot']['total_packets'] / total_s
                stats['hand']['avg_rate'] = \
                    stats['hand']['total_packets'] / total_s
                stats['commands']['avg_rate'] = \
                    stats['commands']['total_packets'] / total_s

            # 내부용 키 제거 후 저장
            save_data = {k: v for k, v in stats.items()
                         if not k.startswith('_')}

            ctrl_dir = self.stats_dir / ctrl_name
            ctrl_dir.mkdir(parents=True, exist_ok=True)
            filepath = ctrl_dir / filename
            with open(filepath, 'w') as f:
                json.dump(save_data, f, indent=2)

            self.get_logger().info(f'Statistics saved: {filepath}')

    def destroy_node(self):
        self.log_status()
        self.save_statistics()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataHealthMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # destroy_node()는 rclpy context 유효 여부와 무관하게 항상 호출.
        # ros2 launch 종료 시 SIGINT 핸들러가 rclpy.shutdown()을 먼저 호출해
        # rclpy.ok() == False가 되므로, 조건 밖에서 save_statistics()를 실행해야 함.
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
