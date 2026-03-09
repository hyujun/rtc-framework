#!/usr/bin/env python3
"""
monitor_data_health_v2.py - v2 (Statistics Collection)

Real-time data health monitoring with statistics collection and JSON export.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool
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

        # Create stats directory
        if self.enable_stats:
            self.stats_dir.mkdir(parents=True, exist_ok=True)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        self.hand_state_sub = self.create_subscription(
            Float64MultiArray, '/hand/joint_states', self.hand_state_callback, 10)

        self.command_sub = self.create_subscription(
            Float64MultiArray, '/forward_position_controller/commands',
            self.command_callback, 10)

        self.estop_sub = self.create_subscription(
            Bool, '/system/estop_status', self.estop_callback, 10)

        # Monitoring timer
        self.timer = self.create_timer(
            1.0 / self.check_rate, self.check_health)

        # Statistics
        self.stats = {
            'start_time': datetime.now().isoformat(),
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
            }
        }

        self.last_robot_time = None
        self.last_hand_time = None
        self.last_command_time = None

        self.robot_timeout_count = 0
        self.hand_timeout_count = 0

        self.get_logger().info('Data Health Monitor v2 started')
        self.get_logger().info(f'Check rate: {self.check_rate} Hz')
        self.get_logger().info(
            f'Timeout threshold: {self.timeout_threshold} s')
        if self.enable_stats:
            self.get_logger().info(f'Statistics output: {self.stats_dir}')

    def joint_state_callback(self, msg):
        now = self.get_clock().now()
        self.last_robot_time = now
        self.stats['robot']['total_packets'] += 1
        self.stats['robot']['last_update'] = datetime.now().isoformat()

    def hand_state_callback(self, msg):
        now = self.get_clock().now()
        self.last_hand_time = now
        self.stats['hand']['total_packets'] += 1
        self.stats['hand']['last_update'] = datetime.now().isoformat()

    def command_callback(self, msg):
        now = self.get_clock().now()
        self.last_command_time = now
        self.stats['commands']['total_packets'] += 1
        self.stats['commands']['last_update'] = datetime.now().isoformat()

    def estop_callback(self, msg):
        if msg.data and not self.stats['estop']['current_status']:
            # E-STOP triggered
            self.stats['estop']['total_triggers'] += 1
            self.stats['estop']['last_trigger'] = datetime.now().isoformat()
            self.get_logger().error('⚠️  E-STOP TRIGGERED!')

        self.stats['estop']['current_status'] = msg.data

    def check_health(self):
        now = self.get_clock().now()

        # Check robot data
        if self.last_robot_time is not None:
            robot_age = (now - self.last_robot_time).nanoseconds / 1e9
            if robot_age > self.timeout_threshold:
                self.robot_timeout_count += 1
                self.stats['robot']['lost_packets'] += 1
                self.get_logger().warn(
                    f'Robot data timeout: {robot_age:.3f}s '
                    f'(count: {self.robot_timeout_count})')

        # Check hand data
        if self.last_hand_time is not None:
            hand_age = (now - self.last_hand_time).nanoseconds / 1e9
            if hand_age > self.timeout_threshold:
                self.hand_timeout_count += 1
                self.stats['hand']['lost_packets'] += 1
                self.get_logger().warn(
                    f'Hand data timeout: {hand_age:.3f}s '
                    f'(count: {self.hand_timeout_count})')

        # Log status every 10 seconds
        if self.stats['robot']['total_packets'] % 100 == 0 and \
           self.stats['robot']['total_packets'] > 0:
            self.log_status()

    def log_status(self):
        robot_packets = self.stats['robot']['total_packets']
        robot_lost = self.stats['robot']['lost_packets']
        hand_packets = self.stats['hand']['total_packets']
        hand_lost = self.stats['hand']['lost_packets']

        robot_loss_rate = (robot_lost / robot_packets *
                           100) if robot_packets > 0 else 0
        hand_loss_rate = (hand_lost / hand_packets *
                          100) if hand_packets > 0 else 0

        self.get_logger().info('=== Data Health Status ===')
        self.get_logger().info(
            f'Robot: {robot_packets} packets, '
            f'{robot_lost} lost ({robot_loss_rate:.2f}%)')
        self.get_logger().info(
            f'Hand: {hand_packets} packets, '
            f'{hand_lost} lost ({hand_loss_rate:.2f}%)')
        self.get_logger().info(
            f'Commands: {self.stats["commands"]["total_packets"]} sent')
        self.get_logger().info(
            f'E-STOP: {"ACTIVE" if self.stats["estop"]["current_status"] else "CLEAR"} '
            f'(triggered {self.stats["estop"]["total_triggers"]} times)')

    def save_statistics(self):
        if not self.enable_stats:
            return

        # Calculate rates
        elapsed = (datetime.now() - datetime.fromisoformat(
            self.stats['start_time'])).total_seconds()

        if elapsed > 0:
            self.stats['robot']['avg_rate'] = \
                self.stats['robot']['total_packets'] / elapsed
            self.stats['hand']['avg_rate'] = \
                self.stats['hand']['total_packets'] / elapsed
            self.stats['commands']['avg_rate'] = \
                self.stats['commands']['total_packets'] / elapsed

        # Save to JSON
        filename = f"health_stats_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        filepath = self.stats_dir / filename

        with open(filepath, 'w') as f:
            json.dump(self.stats, f, indent=2)

        self.get_logger().info(f'Statistics saved to {filepath}')

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
        # Check if context is still valid before cleanup
        try:
            if rclpy.ok():
                node.destroy_node()
                rclpy.shutdown()
        except Exception:
            # Context already shutdown by another node, ignore
            pass


if __name__ == '__main__':
    main()
