#!/usr/bin/env python3

import sys
import json
import numpy as np
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class MotionEditor(QMainWindow):
    """UR5e 50-Pose Motion Editor GUI"""

    def __init__(self):
        super().__init__()
        self.MAX_POSES = 50
        self.poses = [np.zeros(6) for _ in range(self.MAX_POSES)]
        self.pose_names = [f"Pose {i+1}" for i in range(self.MAX_POSES)]
        self.pose_descriptions = ["" for _ in range(self.MAX_POSES)]
        self.current_q = np.zeros(6)
        self.selected_row = 0

        self.init_ui()
        self.setWindowTitle("UR5e Motion Editor - 50 Poses 🎛️")
        self.setGeometry(100, 100, 1000, 700)

    def init_ui(self):
        """UI 초기화"""
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # 상태 표시
        self.status_label = QLabel("🔴 Waiting for robot...")
        self.status_label.setFont(QFont("Arial", 14, QFont.Bold))
        self.status_label.setStyleSheet("padding: 10px; background: #f0f0f0; border-radius: 5px;")
        layout.addWidget(self.status_label)

        # 현재 관절 각도 표시
        joint_group = QGroupBox("Current Joint Angles")
        joint_layout = QHBoxLayout()
        self.joint_labels = []
        for i in range(6):
            label = QLabel(f"J{i+1}: 0.000")
            label.setFont(QFont("Courier", 11))
            self.joint_labels.append(label)
            joint_layout.addWidget(label)
        joint_group.setLayout(joint_layout)
        layout.addWidget(joint_group)

        # 포즈 테이블
        self.pose_table = QTableWidget(self.MAX_POSES, 5)
        self.pose_table.setHorizontalHeaderLabels(["#", "Name", "Status", "Preview", "Description"])
        self.pose_table.setColumnWidth(0, 50)
        self.pose_table.setColumnWidth(1, 150)
        self.pose_table.setColumnWidth(2, 100)
        self.pose_table.setColumnWidth(3, 380)
        self.pose_table.setColumnWidth(4, 250)
        self.pose_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.pose_table.setSelectionMode(QTableWidget.MultiSelection)
        self.pose_table.itemChanged.connect(self._on_item_changed)

        for i in range(self.MAX_POSES):
            # 체크박스를 포함한 번호 셀
            num_item = QTableWidgetItem(str(i+1))
            num_item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            num_item.setCheckState(Qt.Unchecked)
            self.pose_table.setItem(i, 0, num_item)

            self.pose_table.setItem(i, 1, QTableWidgetItem(self.pose_names[i]))
            self.pose_table.setItem(i, 2, QTableWidgetItem("Empty"))
            self.pose_table.setItem(i, 3, QTableWidgetItem("-"))
            desc_item = QTableWidgetItem("")
            self.pose_table.setItem(i, 4, desc_item)

        layout.addWidget(self.pose_table)

        # 체크박스 선택 버튼 행
        check_layout = QHBoxLayout()

        self.select_all_btn = QPushButton("☑ Select All (Saved)")
        self.select_all_btn.setStyleSheet("padding: 6px; font-size: 12px;")
        self.select_all_btn.clicked.connect(self.select_all_poses)

        self.deselect_all_btn = QPushButton("☐ Deselect All")
        self.deselect_all_btn.setStyleSheet("padding: 6px; font-size: 12px;")
        self.deselect_all_btn.clicked.connect(self.deselect_all_poses)

        check_layout.addWidget(self.select_all_btn)
        check_layout.addWidget(self.deselect_all_btn)
        check_layout.addStretch()
        layout.addLayout(check_layout)

        # 버튼 그룹
        btn_layout = QHBoxLayout()

        self.save_btn = QPushButton("💾 Save Current Pose")
        self.save_btn.setStyleSheet("background: #4CAF50; color: white; padding: 10px; font-size: 14px;")
        self.save_btn.clicked.connect(self.save_pose)

        self.load_btn = QPushButton("📤 Load Selected Pose")
        self.load_btn.setStyleSheet("background: #2196F3; color: white; padding: 10px; font-size: 14px;")
        self.load_btn.clicked.connect(self.load_pose)

        self.play_btn = QPushButton("▶️ Play Checked Poses")
        self.play_btn.setStyleSheet("background: #FF9800; color: white; padding: 10px; font-size: 14px;")
        self.play_btn.clicked.connect(self.play_motion)

        self.clear_btn = QPushButton("🗑️ Clear Selected")
        self.clear_btn.setStyleSheet("background: #f44336; color: white; padding: 10px; font-size: 14px;")
        self.clear_btn.clicked.connect(self.clear_pose)

        btn_layout.addWidget(self.save_btn)
        btn_layout.addWidget(self.load_btn)
        btn_layout.addWidget(self.play_btn)
        btn_layout.addWidget(self.clear_btn)
        layout.addLayout(btn_layout)

        # 메뉴바
        menubar = self.menuBar()
        file_menu = menubar.addMenu("File")

        save_action = file_menu.addAction("Save Motion to JSON")
        save_action.triggered.connect(self.save_json)

        load_action = file_menu.addAction("Load Motion from JSON")
        load_action.triggered.connect(self.load_json)

        file_menu.addSeparator()
        exit_action = file_menu.addAction("Exit")
        exit_action.triggered.connect(self.close)

    def _on_item_changed(self, item):
        """Sync in-memory data when user edits Name or Description cells directly."""
        row = item.row()
        col = item.column()
        if col == 1:
            self.pose_names[row] = item.text()
        elif col == 4:
            self.pose_descriptions[row] = item.text()

    def _checked_rows(self):
        """체크된 행 인덱스 목록 반환"""
        return [
            i for i in range(self.MAX_POSES)
            if self.pose_table.item(i, 0).checkState() == Qt.Checked
        ]

    def select_all_poses(self):
        """저장된 포즈만 전체 체크"""
        self.pose_table.itemChanged.disconnect(self._on_item_changed)
        for i in range(self.MAX_POSES):
            item = self.pose_table.item(i, 0)
            if np.linalg.norm(self.poses[i]) > 0.001:
                item.setCheckState(Qt.Checked)
        self.pose_table.itemChanged.connect(self._on_item_changed)

    def deselect_all_poses(self):
        """전체 체크 해제"""
        self.pose_table.itemChanged.disconnect(self._on_item_changed)
        for i in range(self.MAX_POSES):
            self.pose_table.item(i, 0).setCheckState(Qt.Unchecked)
        self.pose_table.itemChanged.connect(self._on_item_changed)

    def update_joints(self, q):
        """ROS에서 받은 관절 각도 업데이트"""
        self.current_q = q.copy()
        for i, val in enumerate(q):
            self.joint_labels[i].setText(f"J{i+1}: {val:.3f}")
        self.status_label.setText(f"🟢 Live - Ready to save")

    def save_pose(self):
        """현재 포즈 저장"""
        selected = self.pose_table.selectedIndexes()
        if not selected:
            row = 0  # 선택 없으면 첫 번째 빈 슬롯
            for i in range(self.MAX_POSES):
                if np.linalg.norm(self.poses[i]) < 0.001:
                    row = i
                    break
        else:
            row = selected[0].row()

        self.poses[row] = self.current_q.copy()
        self.pose_table.item(row, 2).setText("✅ Saved")
        q_deg = np.degrees(self.current_q)
        self.pose_table.item(row, 3).setText(
            f"[{q_deg[0]:.2f}, {q_deg[1]:.2f}, {q_deg[2]:.2f}, "
            f"{q_deg[3]:.2f}, {q_deg[4]:.2f}, {q_deg[5]:.2f}]°"
        )
        self.status_label.setText(f"✅ Saved to Pose {row+1}")

    def load_pose(self):
        """선택된 포즈 로드 (로봇에 전송)"""
        selected = self.pose_table.selectedIndexes()
        if not selected:
            QMessageBox.warning(self, "Warning", "No pose selected!")
            return

        row = selected[0].row()
        if np.linalg.norm(self.poses[row]) < 0.001:
            QMessageBox.warning(self, "Warning", f"Pose {row+1} is empty!")
            return

        # ROS 퍼블리시 (외부 노드에서 처리)
        self.status_label.setText(f"📤 Loading Pose {row+1}...")
        # 실제 ROS 통신은 ROSNode에서 처리
        if hasattr(self, 'ros_node'):
            self.ros_node.publish_pose(self.poses[row])

    def play_motion(self):
        """체크된 포즈 순차 재생"""
        checked = self._checked_rows()

        if not checked:
            QMessageBox.warning(self, "Warning", "No poses checked!\nUse the checkboxes to select poses to play.")
            return

        valid_poses = [i for i in checked if np.linalg.norm(self.poses[i]) > 0.001]

        if not valid_poses:
            QMessageBox.warning(self, "Warning", "Checked poses are all empty!")
            return

        reply = QMessageBox.question(
            self, "Confirm",
            f"Play {len(valid_poses)} checked poses in sequence?\n"
            f"Poses: {[i+1 for i in valid_poses]}",
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self.status_label.setText(f"▶️ Playing {len(valid_poses)} poses...")
            # 실제 재생은 ROSNode에서 처리
            if hasattr(self, 'ros_node'):
                self.ros_node.play_sequence([self.poses[i] for i in valid_poses])

    def clear_pose(self):
        """선택된 포즈 삭제"""
        selected = self.pose_table.selectedIndexes()
        if not selected:
            return

        for idx in selected:
            row = idx.row()
            self.poses[row] = np.zeros(6)
            self.pose_descriptions[row] = ""
            self.pose_table.item(row, 2).setText("Empty")
            self.pose_table.item(row, 3).setText("-")
            self.pose_table.item(row, 4).setText("")

        self.status_label.setText("🗑️ Cleared selected poses")

    def save_json(self):
        """모션 JSON 저장"""
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Motion", "", "JSON Files (*.json)")

        if filename:
            data = {
                "num_poses": self.MAX_POSES,
                "poses": {f"pose_{i}": self.poses[i].tolist()
                         for i in range(self.MAX_POSES)},
                "names": self.pose_names,
                "descriptions": self.pose_descriptions
            }
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            self.status_label.setText(f"💾 Saved to {filename}")

    def load_json(self):
        """모션 JSON 로드"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Load Motion", "", "JSON Files (*.json)")

        if filename:
            try:
                with open(filename, 'r') as f:
                    data = json.load(f)

                descriptions = data.get('descriptions', [""] * self.MAX_POSES)
                for i in range(min(self.MAX_POSES, len(data.get('poses', {})))):
                    key = f"pose_{i}"
                    if key in data['poses']:
                        self.poses[i] = np.array(data['poses'][key])
                        desc = descriptions[i] if i < len(descriptions) else ""
                        self.pose_descriptions[i] = desc
                        self.pose_table.item(i, 4).setText(desc)
                        if np.linalg.norm(self.poses[i]) > 0.001:
                            self.pose_table.item(i, 2).setText("✅ Saved")
                            q_deg = np.degrees(self.poses[i])
                            self.pose_table.item(i, 3).setText(
                                f"[{q_deg[0]:.2f}, {q_deg[1]:.2f}, {q_deg[2]:.2f}, "
                                f"{q_deg[3]:.2f}, {q_deg[4]:.2f}, {q_deg[5]:.2f}]°"
                            )

                self.status_label.setText(f"📂 Loaded from {filename}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load: {str(e)}")


class ROSNode(Node):
    """ROS2 통신 노드"""

    def __init__(self, gui):
        super().__init__('motion_editor_node')
        self.gui = gui
        gui.ros_node = self

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, qos)

        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', qos)

        self.get_logger().info("Motion Editor ROS Node started")

    def joint_callback(self, msg):
        """관절 상태 콜백"""
        if len(msg.position) >= 6:
            q = np.array(msg.position[:6])
            self.gui.update_joints(q)

    def publish_pose(self, pose):
        """포즈 퍼블리시"""
        msg = Float64MultiArray()
        msg.data = pose.tolist()
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Published pose: {pose}")

    def play_sequence(self, poses):
        """포즈 시퀀스 재생"""
        import time
        for i, pose in enumerate(poses):
            self.publish_pose(pose)
            self.get_logger().info(f"Playing pose {i+1}/{len(poses)}")
            time.sleep(2.0)  # 2초 간격


def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    gui = MotionEditor()
    gui.show()

    ros_node = ROSNode(gui)

    # ROS2 spin을 Qt 타이머로 통합
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    timer.start(10)  # 100Hz

    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
