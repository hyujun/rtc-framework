#!/usr/bin/env python3

import sys
import json
import signal
import numpy as np
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QColor, QBrush

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# 컬럼 인덱스
COL_NUM     = 0   # # (체크박스)
COL_NAME    = 1   # Name
COL_STATUS  = 2   # Status
COL_PREVIEW = 3   # Preview (읽기전용)
COL_TRAJ    = 4   # Traj (s) — 궤적 실행 시간
COL_WAIT    = 5   # Wait (s) — 도달 후 대기 시간
COL_DESC    = 6   # Description
NUM_COLS    = 7


class MotionEditor(QMainWindow):
    """UR5e 50-Pose Motion Editor GUI"""

    def __init__(self):
        super().__init__()
        self.MAX_POSES = 50
        self.poses = [np.zeros(6) for _ in range(self.MAX_POSES)]
        self.pose_names = [f"Pose {i+1}" for i in range(self.MAX_POSES)]
        self.pose_descriptions = ["" for _ in range(self.MAX_POSES)]
        self.current_q = np.zeros(6)
        self._play_queue = []
        self._play_step = 0
        # 단발(single-shot) 타이머로 per-row 딜레이 구현
        self._play_timer = QTimer()
        self._play_timer.setSingleShot(True)
        self._play_timer.timeout.connect(self._play_next)

        self.init_ui()
        self.setWindowTitle("UR5e Motion Editor - 50 Poses 🎛️")
        self.setGeometry(100, 100, 1100, 720)

    # ──────────────────────────── 스핀박스 팩토리 ────────────────────────────

    def _make_traj_spin(self, value=2.0):
        """Traj(s) 스핀박스 생성"""
        spin = QDoubleSpinBox()
        spin.setRange(0.1, 60.0)
        spin.setSingleStep(0.5)
        spin.setValue(value)
        spin.setDecimals(1)
        spin.setFrame(False)
        spin.setToolTip("Trajectory duration (s): time allowed for robot to reach this pose")
        return spin

    def _make_wait_spin(self, value=0.0):
        """Wait(s) 스핀박스 생성"""
        spin = QDoubleSpinBox()
        spin.setRange(0.0, 60.0)
        spin.setSingleStep(0.1)
        spin.setValue(value)
        spin.setDecimals(1)
        spin.setFrame(False)
        spin.setToolTip("Wait duration (s): extra hold time after reaching this pose")
        return spin

    def _get_row_timing(self, row):
        """row의 (traj_s, wait_s) 반환"""
        traj_spin = self.pose_table.cellWidget(row, COL_TRAJ)
        wait_spin = self.pose_table.cellWidget(row, COL_WAIT)
        return (traj_spin.value() if traj_spin else 2.0,
                wait_spin.value() if wait_spin else 0.0)

    # ──────────────────────────── UI 초기화 ────────────────────────────

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

        # 포즈 테이블 (7 컬럼)
        self.pose_table = QTableWidget(self.MAX_POSES, NUM_COLS)
        self.pose_table.setHorizontalHeaderLabels(
            ["#", "Name", "Status", "Preview", "Traj (s)", "Wait (s)", "Description"])
        self.pose_table.setColumnWidth(COL_NUM,     50)
        self.pose_table.setColumnWidth(COL_NAME,   150)
        self.pose_table.setColumnWidth(COL_STATUS,  90)
        self.pose_table.setColumnWidth(COL_PREVIEW,320)
        self.pose_table.setColumnWidth(COL_TRAJ,    75)
        self.pose_table.setColumnWidth(COL_WAIT,    75)
        self.pose_table.setColumnWidth(COL_DESC,   200)
        self.pose_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.pose_table.setSelectionMode(QTableWidget.MultiSelection)
        self.pose_table.itemChanged.connect(self._on_item_changed)

        for i in range(self.MAX_POSES):
            self._init_row(i)

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
        self.save_btn.setStyleSheet("background: #4CAF50; color: white; padding: 10px; font-size: 13px;")
        self.save_btn.clicked.connect(self.save_pose)

        self.load_btn = QPushButton("📤 Load Selected Pose")
        self.load_btn.setStyleSheet("background: #2196F3; color: white; padding: 10px; font-size: 13px;")
        self.load_btn.clicked.connect(self.load_pose)

        self.insert_btn = QPushButton("➕ Insert Empty Row")
        self.insert_btn.setStyleSheet("background: #9C27B0; color: white; padding: 10px; font-size: 13px;")
        self.insert_btn.setToolTip(
            "Insert an empty row after the selected row.\n"
            "All rows below shift down; the last row is dropped.")
        self.insert_btn.clicked.connect(self.insert_row)

        self.play_btn = QPushButton("▶️ Play Checked Poses")
        self.play_btn.setStyleSheet("background: #FF9800; color: white; padding: 10px; font-size: 13px;")
        self.play_btn.clicked.connect(self.play_motion)

        self.stop_btn = QPushButton("⏹ Stop")
        self.stop_btn.setStyleSheet("background: #9E9E9E; color: white; padding: 10px; font-size: 13px;")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.stop_motion)

        self.clear_btn = QPushButton("🗑️ Clear Selected")
        self.clear_btn.setStyleSheet("background: #f44336; color: white; padding: 10px; font-size: 13px;")
        self.clear_btn.clicked.connect(self.clear_pose)

        btn_layout.addWidget(self.save_btn)
        btn_layout.addWidget(self.load_btn)
        btn_layout.addWidget(self.insert_btn)
        btn_layout.addWidget(self.play_btn)
        btn_layout.addWidget(self.stop_btn)
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

    # ──────────────────────────── 행 초기화 ────────────────────────────

    def _init_row(self, i, traj=2.0, wait=0.0):
        """i번째 행 셀·위젯 초기화"""
        # # + 체크박스
        num_item = QTableWidgetItem(str(i + 1))
        num_item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
        num_item.setCheckState(Qt.Unchecked)
        self.pose_table.setItem(i, COL_NUM, num_item)

        # Name
        name = self.pose_names[i] if i < len(self.pose_names) else f"Pose {i+1}"
        self.pose_table.setItem(i, COL_NAME, QTableWidgetItem(name))

        # Status
        self.pose_table.setItem(i, COL_STATUS, QTableWidgetItem("Empty"))

        # Preview (읽기전용)
        preview_item = QTableWidgetItem("-")
        preview_item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
        preview_item.setToolTip("Read-only: updated automatically when pose is saved")
        self.pose_table.setItem(i, COL_PREVIEW, preview_item)

        # Traj / Wait 스핀박스
        self.pose_table.setCellWidget(i, COL_TRAJ, self._make_traj_spin(traj))
        self.pose_table.setCellWidget(i, COL_WAIT, self._make_wait_spin(wait))

        # Description
        desc = self.pose_descriptions[i] if i < len(self.pose_descriptions) else ""
        self.pose_table.setItem(i, COL_DESC, QTableWidgetItem(desc))

    # ──────────────────────────── 이벤트 핸들러 ────────────────────────────

    def _on_item_changed(self, item):
        """Name / Description 셀 편집 시 메모리 동기화"""
        row = item.row()
        col = item.column()
        if col == COL_NAME and row < len(self.pose_names):
            self.pose_names[row] = item.text()
        elif col == COL_DESC and row < len(self.pose_descriptions):
            self.pose_descriptions[row] = item.text()

    def _checked_rows(self):
        """체크된 행 인덱스 목록 반환"""
        return [i for i in range(self.MAX_POSES)
                if self.pose_table.item(i, COL_NUM).checkState() == Qt.Checked]

    def select_all_poses(self):
        """저장된 포즈만 전체 체크"""
        self.pose_table.itemChanged.disconnect(self._on_item_changed)
        for i in range(self.MAX_POSES):
            if np.linalg.norm(self.poses[i]) > 0.001:
                self.pose_table.item(i, COL_NUM).setCheckState(Qt.Checked)
        self.pose_table.itemChanged.connect(self._on_item_changed)

    def deselect_all_poses(self):
        """전체 체크 해제"""
        self.pose_table.itemChanged.disconnect(self._on_item_changed)
        for i in range(self.MAX_POSES):
            self.pose_table.item(i, COL_NUM).setCheckState(Qt.Unchecked)
        self.pose_table.itemChanged.connect(self._on_item_changed)

    # ──────────────────────────── 포즈 조작 ────────────────────────────

    def update_joints(self, q):
        """ROS에서 받은 관절 각도 업데이트"""
        self.current_q = q.copy()
        for i, val in enumerate(q):
            self.joint_labels[i].setText(f"J{i+1}: {np.degrees(val):.2f}°")
        self.status_label.setText("🟢 Live - Ready to save")

    def save_pose(self):
        """현재 포즈 저장"""
        selected = self.pose_table.selectedIndexes()
        if not selected:
            row = next((i for i in range(self.MAX_POSES)
                        if np.linalg.norm(self.poses[i]) < 0.001), 0)
        else:
            row = selected[0].row()

        self.poses[row] = self.current_q.copy()
        self.pose_table.item(row, COL_STATUS).setText("✅ Saved")
        q_deg = np.degrees(self.current_q)
        preview = self.pose_table.item(row, COL_PREVIEW)
        preview.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
        preview.setText(
            f"[{q_deg[0]:.2f}, {q_deg[1]:.2f}, {q_deg[2]:.2f}, "
            f"{q_deg[3]:.2f}, {q_deg[4]:.2f}, {q_deg[5]:.2f}]°"
        )
        self.pose_table.clearSelection()
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
        self.status_label.setText(f"📤 Loading Pose {row+1}...")
        if hasattr(self, 'ros_node'):
            self.ros_node.publish_pose(self.poses[row])

    def insert_row(self):
        """선택된 행 바로 아래에 빈 행 삽입 (마지막 행은 제거됨)"""
        selected = self.pose_table.selectedIndexes()
        if not selected:
            QMessageBox.warning(self, "Warning",
                                "No row selected!\nClick a row to select it first.")
            return

        insert_after = selected[0].row()
        ins = insert_after + 1  # 삽입될 위치

        # 마지막 행에 데이터가 있으면 경고
        last = self.MAX_POSES - 1
        if np.linalg.norm(self.poses[last]) > 0.001:
            reply = QMessageBox.question(
                self, "Confirm",
                f"Inserting a row will drop Pose {last+1} (last row has saved data).\nContinue?",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply != QMessageBox.Yes:
                return

        # 현재 스핀박스 값 수집 (shift 전)
        traj_vals = [self._get_row_timing(i)[0] for i in range(self.MAX_POSES)]
        wait_vals = [self._get_row_timing(i)[1] for i in range(self.MAX_POSES)]

        # 데이터 배열 shift — ins 위치에 빈 항목 삽입, 마지막 제거
        self.poses.insert(ins, np.zeros(6))
        self.poses.pop()
        self.pose_names.insert(ins, f"Pose {ins+1}")
        self.pose_names.pop()
        self.pose_descriptions.insert(ins, "")
        self.pose_descriptions.pop()
        traj_vals.insert(ins, 2.0)
        traj_vals.pop()
        wait_vals.insert(ins, 0.0)
        wait_vals.pop()

        self._rebuild_table(traj_vals, wait_vals)
        self.status_label.setText(f"➕ Inserted empty row after row {insert_after+1}")

    def _rebuild_table(self, traj_vals=None, wait_vals=None):
        """테이블 전체 재구성 (insert 후 등 데이터 배열과 동기화)"""
        if traj_vals is None:
            traj_vals = [self._get_row_timing(i)[0] for i in range(self.MAX_POSES)]
        if wait_vals is None:
            wait_vals = [self._get_row_timing(i)[1] for i in range(self.MAX_POSES)]

        self.pose_table.itemChanged.disconnect(self._on_item_changed)

        for i in range(self.MAX_POSES):
            has_pose = np.linalg.norm(self.poses[i]) > 0.001

            # # 셀 — 번호 업데이트, 체크 해제
            num_item = self.pose_table.item(i, COL_NUM)
            num_item.setText(str(i + 1))
            num_item.setCheckState(Qt.Unchecked)

            # Name
            self.pose_table.item(i, COL_NAME).setText(self.pose_names[i])

            # Status
            self.pose_table.item(i, COL_STATUS).setText("✅ Saved" if has_pose else "Empty")

            # Preview (읽기전용 유지)
            preview = self.pose_table.item(i, COL_PREVIEW)
            preview.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
            if has_pose:
                q_deg = np.degrees(self.poses[i])
                preview.setText(
                    f"[{q_deg[0]:.2f}, {q_deg[1]:.2f}, {q_deg[2]:.2f}, "
                    f"{q_deg[3]:.2f}, {q_deg[4]:.2f}, {q_deg[5]:.2f}]°"
                )
            else:
                preview.setText("-")

            # Traj / Wait 스핀박스 값 업데이트
            traj_spin = self.pose_table.cellWidget(i, COL_TRAJ)
            wait_spin = self.pose_table.cellWidget(i, COL_WAIT)
            if traj_spin:
                traj_spin.setValue(traj_vals[i])
            if wait_spin:
                wait_spin.setValue(wait_vals[i])

            # Description
            self.pose_table.item(i, COL_DESC).setText(self.pose_descriptions[i])

        self.pose_table.itemChanged.connect(self._on_item_changed)

    def clear_pose(self):
        """선택된 포즈 삭제"""
        selected = self.pose_table.selectedIndexes()
        if not selected:
            return

        rows = set(idx.row() for idx in selected)
        for row in rows:
            self.poses[row] = np.zeros(6)
            self.pose_descriptions[row] = ""
            self.pose_table.item(row, COL_STATUS).setText("Empty")
            preview = self.pose_table.item(row, COL_PREVIEW)
            preview.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
            preview.setText("-")
            self.pose_table.item(row, COL_DESC).setText("")

        self.pose_table.clearSelection()
        self.status_label.setText("🗑️ Cleared selected poses")

    # ──────────────────────────── 재생 ────────────────────────────

    def play_motion(self):
        """체크된 포즈 순차 재생 (per-row 타이밍 적용)"""
        checked = self._checked_rows()
        if not checked:
            QMessageBox.warning(self, "Warning",
                                "No poses checked!\nUse the checkboxes to select poses to play.")
            return

        valid = [(i, self.poses[i]) for i in checked
                 if np.linalg.norm(self.poses[i]) > 0.001]
        if not valid:
            QMessageBox.warning(self, "Warning", "Checked poses are all empty!")
            return

        # 타이밍 미리보기 포함 확인 다이얼로그
        timing_preview = "\n".join(
            f"  Pose {row+1}: traj={self._get_row_timing(row)[0]:.1f}s + "
            f"wait={self._get_row_timing(row)[1]:.1f}s"
            for row, _ in valid
        )
        reply = QMessageBox.question(
            self, "Confirm",
            f"Play {len(valid)} checked poses in sequence?\n"
            f"Poses: {[r+1 for r, _ in valid]}\n\n{timing_preview}",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            return

        self._play_queue = valid
        self._play_step = 0
        self.play_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self._play_next()

    def _play_next(self):
        """다음 포즈 전송 — per-row 딜레이(traj+wait)로 단발 타이머 재시작"""
        # 이전 행 하이라이트 해제
        if self._play_step > 0:
            prev_row = self._play_queue[self._play_step - 1][0]
            self._set_row_highlight(prev_row, False)

        # 재생 완료
        if self._play_step >= len(self._play_queue):
            self.play_btn.setEnabled(True)
            self.stop_btn.setEnabled(False)
            self.status_label.setText(f"✅ Motion complete ({len(self._play_queue)} poses)")
            return

        row, pose = self._play_queue[self._play_step]
        self._set_row_highlight(row, True)
        self.pose_table.scrollToItem(self.pose_table.item(row, COL_NUM))

        traj, wait = self._get_row_timing(row)
        delay_ms = max(100, int((traj + wait) * 1000))
        self.status_label.setText(
            f"▶️ Playing Pose {row+1}  ({self._play_step+1}/{len(self._play_queue)})  "
            f"[traj={traj:.1f}s + wait={wait:.1f}s]"
        )

        if hasattr(self, 'ros_node'):
            self.ros_node.publish_pose(pose)

        self._play_step += 1
        self._play_timer.start(delay_ms)

    def stop_motion(self):
        """재생 중지"""
        self._play_timer.stop()
        if 0 < self._play_step <= len(self._play_queue):
            self._set_row_highlight(self._play_queue[self._play_step - 1][0], False)
        self._play_queue = []
        self._play_step = 0
        self.play_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.status_label.setText("⏹ Stopped")

    def _set_row_highlight(self, row, active):
        """행 배경색 하이라이트"""
        color = QColor("#FFF176") if active else QColor("white")
        brush = QBrush(color)
        for col in range(self.pose_table.columnCount()):
            item = self.pose_table.item(row, col)
            if item:
                item.setBackground(brush)

    # ──────────────────────────── JSON 저장/로드 ────────────────────────────

    def save_json(self):
        """모션 JSON 저장 (traj/wait 시간 포함)"""
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Motion", "", "JSON Files (*.json)")
        if not filename:
            return

        traj_times, wait_times = [], []
        for i in range(self.MAX_POSES):
            t, w = self._get_row_timing(i)
            traj_times.append(t)
            wait_times.append(w)

        data = {
            "num_poses": self.MAX_POSES,
            "poses": {f"pose_{i}": self.poses[i].tolist() for i in range(self.MAX_POSES)},
            "names": self.pose_names,
            "descriptions": self.pose_descriptions,
            "traj_times": traj_times,
            "wait_times": wait_times,
        }
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        self.status_label.setText(f"💾 Saved to {filename}")

    def load_json(self):
        """모션 JSON 로드 (traj/wait 시간 복원, 구버전 JSON 호환)"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Load Motion", "", "JSON Files (*.json)")
        if not filename:
            return

        try:
            with open(filename, 'r') as f:
                data = json.load(f)

            descriptions = data.get('descriptions', [""] * self.MAX_POSES)
            traj_times   = data.get('traj_times',   [2.0] * self.MAX_POSES)
            wait_times   = data.get('wait_times',   [0.0] * self.MAX_POSES)

            self.pose_table.itemChanged.disconnect(self._on_item_changed)

            for i in range(min(self.MAX_POSES, len(data.get('poses', {})))):
                key = f"pose_{i}"
                if key not in data['poses']:
                    continue

                self.poses[i] = np.array(data['poses'][key])
                desc = descriptions[i] if i < len(descriptions) else ""
                self.pose_descriptions[i] = desc
                self.pose_table.item(i, COL_DESC).setText(desc)

                traj = traj_times[i] if i < len(traj_times) else 2.0
                wait = wait_times[i] if i < len(wait_times) else 0.0
                traj_spin = self.pose_table.cellWidget(i, COL_TRAJ)
                wait_spin = self.pose_table.cellWidget(i, COL_WAIT)
                if traj_spin:
                    traj_spin.setValue(traj)
                if wait_spin:
                    wait_spin.setValue(wait)

                if np.linalg.norm(self.poses[i]) > 0.001:
                    self.pose_table.item(i, COL_STATUS).setText("✅ Saved")
                    q_deg = np.degrees(self.poses[i])
                    preview = self.pose_table.item(i, COL_PREVIEW)
                    preview.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
                    preview.setText(
                        f"[{q_deg[0]:.2f}, {q_deg[1]:.2f}, {q_deg[2]:.2f}, "
                        f"{q_deg[3]:.2f}, {q_deg[4]:.2f}, {q_deg[5]:.2f}]°"
                    )

            self.pose_table.itemChanged.connect(self._on_item_changed)
            self.status_label.setText(f"📂 Loaded from {filename}")

        except Exception as e:
            # 시그널 연결 복구
            try:
                self.pose_table.itemChanged.connect(self._on_item_changed)
            except Exception:
                pass
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
            Float64MultiArray, '/target_joint_positions', qos)

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


def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    signal.signal(signal.SIGINT, lambda *_: app.quit())

    gui = MotionEditor()
    gui.show()

    ros_node = ROSNode(gui)

    # ROS2 spin을 Qt 타이머로 통합
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0) if rclpy.ok() else None)
    timer.start(10)  # 100Hz

    exit_code = app.exec_()

    timer.stop()
    ros_node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
