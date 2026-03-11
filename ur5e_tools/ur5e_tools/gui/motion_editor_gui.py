#!/usr/bin/env python3

import os
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


class MotionTab(QWidget):
    """단일 모션 파일을 관리하는 탭 위젯"""

    MAX_POSES = 50

    def __init__(self, parent_editor, file_path=None):
        super().__init__()
        self.parent_editor = parent_editor
        self._current_file = file_path
        self._modified = False

        self.poses = [np.zeros(6) for _ in range(self.MAX_POSES)]
        self.pose_names = [f"Pose {i+1}" for i in range(self.MAX_POSES)]
        self.pose_descriptions = ["" for _ in range(self.MAX_POSES)]

        self._init_table()

    # ──────────────────────────── 스핀박스 팩토리 ────────────────────────────

    def _make_traj_spin(self, value=2.0):
        spin = QDoubleSpinBox()
        spin.setRange(0.1, 60.0)
        spin.setSingleStep(0.5)
        spin.setValue(value)
        spin.setDecimals(1)
        spin.setFrame(False)
        spin.setToolTip("Trajectory duration (s): time allowed for robot to reach this pose")
        spin.valueChanged.connect(self._mark_modified)
        return spin

    def _make_wait_spin(self, value=0.0):
        spin = QDoubleSpinBox()
        spin.setRange(0.0, 60.0)
        spin.setSingleStep(0.1)
        spin.setValue(value)
        spin.setDecimals(1)
        spin.setFrame(False)
        spin.setToolTip("Wait duration (s): extra hold time after reaching this pose")
        spin.valueChanged.connect(self._mark_modified)
        return spin

    def _get_row_timing(self, row):
        traj_spin = self.pose_table.cellWidget(row, COL_TRAJ)
        wait_spin = self.pose_table.cellWidget(row, COL_WAIT)
        return (traj_spin.value() if traj_spin else 2.0,
                wait_spin.value() if wait_spin else 0.0)

    # ──────────────────────────── 테이블 초기화 ────────────────────────────

    def _init_table(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.pose_table = QTableWidget(self.MAX_POSES, NUM_COLS)
        self.pose_table.setHorizontalHeaderLabels(
            ["#", "Name", "Status", "Preview", "Traj (s)", "Wait (s)", "Description"])
        self.pose_table.setColumnWidth(COL_NUM,     50)
        self.pose_table.setColumnWidth(COL_NAME,   150)
        self.pose_table.setColumnWidth(COL_STATUS,  90)
        self.pose_table.setColumnWidth(COL_PREVIEW, 320)
        self.pose_table.setColumnWidth(COL_TRAJ,    75)
        self.pose_table.setColumnWidth(COL_WAIT,    75)
        self.pose_table.setColumnWidth(COL_DESC,   200)
        self.pose_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.pose_table.setSelectionMode(QTableWidget.MultiSelection)
        self.pose_table.itemChanged.connect(self._on_item_changed)

        for i in range(self.MAX_POSES):
            self._init_row(i)

        layout.addWidget(self.pose_table)

    def _init_row(self, i, traj=2.0, wait=0.0):
        num_item = QTableWidgetItem(str(i + 1))
        num_item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
        num_item.setCheckState(Qt.Unchecked)
        self.pose_table.setItem(i, COL_NUM, num_item)

        name = self.pose_names[i] if i < len(self.pose_names) else f"Pose {i+1}"
        self.pose_table.setItem(i, COL_NAME, QTableWidgetItem(name))

        self.pose_table.setItem(i, COL_STATUS, QTableWidgetItem("Empty"))

        preview_item = QTableWidgetItem("-")
        preview_item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
        preview_item.setToolTip("Read-only: updated automatically when pose is saved")
        self.pose_table.setItem(i, COL_PREVIEW, preview_item)

        self.pose_table.setCellWidget(i, COL_TRAJ, self._make_traj_spin(traj))
        self.pose_table.setCellWidget(i, COL_WAIT, self._make_wait_spin(wait))

        desc = self.pose_descriptions[i] if i < len(self.pose_descriptions) else ""
        self.pose_table.setItem(i, COL_DESC, QTableWidgetItem(desc))

    # ──────────────────────────── 이벤트 핸들러 ────────────────────────────

    def _on_item_changed(self, item):
        row = item.row()
        col = item.column()
        if col == COL_NAME and row < len(self.pose_names):
            self.pose_names[row] = item.text()
            self._mark_modified()
        elif col == COL_DESC and row < len(self.pose_descriptions):
            self.pose_descriptions[row] = item.text()
            self._mark_modified()

    def _mark_modified(self, _value=None):
        if not self._modified:
            self._modified = True
            self.parent_editor.update_tab_title(self)

    def _mark_saved(self):
        self._modified = False
        self.parent_editor.update_tab_title(self)

    def _checked_rows(self):
        return [i for i in range(self.MAX_POSES)
                if self.pose_table.item(i, COL_NUM).checkState() == Qt.Checked]

    def select_all_poses(self):
        self.pose_table.itemChanged.disconnect(self._on_item_changed)
        for i in range(self.MAX_POSES):
            if np.linalg.norm(self.poses[i]) > 0.001:
                self.pose_table.item(i, COL_NUM).setCheckState(Qt.Checked)
        self.pose_table.itemChanged.connect(self._on_item_changed)

    def deselect_all_poses(self):
        self.pose_table.itemChanged.disconnect(self._on_item_changed)
        for i in range(self.MAX_POSES):
            self.pose_table.item(i, COL_NUM).setCheckState(Qt.Unchecked)
        self.pose_table.itemChanged.connect(self._on_item_changed)

    # ──────────────────────────── 포즈 조작 ────────────────────────────

    def save_pose(self, current_q):
        selected = self.pose_table.selectedIndexes()
        if not selected:
            row = next((i for i in range(self.MAX_POSES)
                        if np.linalg.norm(self.poses[i]) < 0.001), 0)
        else:
            row = selected[0].row()

        self.poses[row] = current_q.copy()
        self.pose_table.item(row, COL_STATUS).setText("✅ Saved")
        q_deg = np.degrees(current_q)
        preview = self.pose_table.item(row, COL_PREVIEW)
        preview.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
        preview.setText(
            f"[{q_deg[0]:.2f}, {q_deg[1]:.2f}, {q_deg[2]:.2f}, "
            f"{q_deg[3]:.2f}, {q_deg[4]:.2f}, {q_deg[5]:.2f}]°"
        )
        self.pose_table.clearSelection()
        self._mark_modified()
        return row

    def load_pose(self):
        selected = self.pose_table.selectedIndexes()
        if not selected:
            QMessageBox.warning(self, "Warning", "No pose selected!")
            return None
        row = selected[0].row()
        if np.linalg.norm(self.poses[row]) < 0.001:
            QMessageBox.warning(self, "Warning", f"Pose {row+1} is empty!")
            return None
        return row

    def insert_row(self):
        selected = self.pose_table.selectedIndexes()
        if not selected:
            QMessageBox.warning(self, "Warning",
                                "No row selected!\nClick a row to select it first.")
            return None

        insert_after = selected[0].row()
        ins = insert_after + 1

        last = self.MAX_POSES - 1
        if np.linalg.norm(self.poses[last]) > 0.001:
            reply = QMessageBox.question(
                self, "Confirm",
                f"Inserting a row will drop Pose {last+1} (last row has saved data).\nContinue?",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply != QMessageBox.Yes:
                return None

        traj_vals = [self._get_row_timing(i)[0] for i in range(self.MAX_POSES)]
        wait_vals = [self._get_row_timing(i)[1] for i in range(self.MAX_POSES)]

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
        self._mark_modified()
        return insert_after

    def insert_rows_at(self, position, row_data_list):
        """지정된 위치에 여러 row 데이터를 삽입 (탭 간 복사/붙여넣기용)"""
        n = len(row_data_list)
        if n == 0:
            return

        traj_vals = [self._get_row_timing(i)[0] for i in range(self.MAX_POSES)]
        wait_vals = [self._get_row_timing(i)[1] for i in range(self.MAX_POSES)]

        # 마지막 n개 row에 데이터가 있으면 경고
        lost_rows = []
        for i in range(self.MAX_POSES - n, self.MAX_POSES):
            if i >= 0 and np.linalg.norm(self.poses[i]) > 0.001:
                lost_rows.append(i + 1)
        if lost_rows:
            reply = QMessageBox.question(
                self, "Confirm",
                f"Inserting {n} row(s) will drop data in row(s): {lost_rows}.\nContinue?",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply != QMessageBox.Yes:
                return

        ins = position + 1
        for j, rd in enumerate(row_data_list):
            idx = ins + j
            self.poses.insert(idx, rd['pose'].copy())
            self.poses.pop()
            self.pose_names.insert(idx, rd.get('name', f"Pose {idx+1}"))
            self.pose_names.pop()
            self.pose_descriptions.insert(idx, rd.get('description', ''))
            self.pose_descriptions.pop()
            traj_vals.insert(idx, rd.get('traj', 2.0))
            traj_vals.pop()
            wait_vals.insert(idx, rd.get('wait', 0.0))
            wait_vals.pop()

        self._rebuild_table(traj_vals, wait_vals)
        self._mark_modified()

    def delete_rows(self):
        """선택된 row 삭제 — 아래 데이터를 위로 당기고 빈 row를 끝에 추가"""
        selected = self.pose_table.selectedIndexes()
        if not selected:
            return 0

        rows = sorted(set(idx.row() for idx in selected), reverse=True)

        traj_vals = [self._get_row_timing(i)[0] for i in range(self.MAX_POSES)]
        wait_vals = [self._get_row_timing(i)[1] for i in range(self.MAX_POSES)]

        for row in rows:
            self.poses.pop(row)
            self.pose_names.pop(row)
            self.pose_descriptions.pop(row)
            traj_vals.pop(row)
            wait_vals.pop(row)

        deleted_count = len(rows)
        while len(self.poses) < self.MAX_POSES:
            idx = len(self.poses)
            self.poses.append(np.zeros(6))
            self.pose_names.append(f"Pose {idx+1}")
            self.pose_descriptions.append("")
            traj_vals.append(2.0)
            wait_vals.append(0.0)

        self._rebuild_table(traj_vals, wait_vals)
        self.pose_table.clearSelection()
        self._mark_modified()
        return deleted_count

    def get_selected_row_data(self):
        """선택된 row들의 데이터를 리스트로 반환 (복사용)"""
        selected = self.pose_table.selectedIndexes()
        if not selected:
            return []

        rows = sorted(set(idx.row() for idx in selected))
        data = []
        for row in rows:
            traj, wait = self._get_row_timing(row)
            data.append({
                'pose': self.poses[row].copy(),
                'name': self.pose_names[row],
                'description': self.pose_descriptions[row],
                'traj': traj,
                'wait': wait,
            })
        return data

    def _rebuild_table(self, traj_vals=None, wait_vals=None):
        if traj_vals is None:
            traj_vals = [self._get_row_timing(i)[0] for i in range(self.MAX_POSES)]
        if wait_vals is None:
            wait_vals = [self._get_row_timing(i)[1] for i in range(self.MAX_POSES)]

        self.pose_table.itemChanged.disconnect(self._on_item_changed)

        for i in range(self.MAX_POSES):
            has_pose = np.linalg.norm(self.poses[i]) > 0.001

            num_item = self.pose_table.item(i, COL_NUM)
            num_item.setText(str(i + 1))
            num_item.setCheckState(Qt.Unchecked)

            # Name — renumbering
            self.pose_names[i] = f"Pose {i+1}"
            self.pose_table.item(i, COL_NAME).setText(self.pose_names[i])

            # Status
            self.pose_table.item(i, COL_STATUS).setText("✅ Saved" if has_pose else "Empty")

            # Preview
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

            # Traj / Wait
            traj_spin = self.pose_table.cellWidget(i, COL_TRAJ)
            wait_spin = self.pose_table.cellWidget(i, COL_WAIT)
            if traj_spin:
                traj_spin.setValue(traj_vals[i])
            if wait_spin:
                wait_spin.setValue(wait_vals[i])

            # Description
            self.pose_table.item(i, COL_DESC).setText(self.pose_descriptions[i])

        self.pose_table.itemChanged.connect(self._on_item_changed)

    def set_row_highlight(self, row, active):
        color = QColor("#FFF176") if active else QColor("white")
        brush = QBrush(color)
        for col in range(self.pose_table.columnCount()):
            item = self.pose_table.item(row, col)
            if item:
                item.setBackground(brush)

    # ──────────────────────────── JSON 저장/로드 ────────────────────────────

    def save_json(self, filename=None):
        if filename is None:
            default_name = self._current_file or ""
            filename, _ = QFileDialog.getSaveFileName(
                self, "Save Motion", default_name,
                "JSON Files (*.json);;All Files (*)")
            if not filename:
                return None

        # 확장자 자동 추가
        if not os.path.splitext(filename)[1]:
            filename += ".json"

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

        self._current_file = filename
        self._mark_saved()
        return filename

    def load_from_json(self, filename):
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
        self._current_file = filename
        self._modified = False

    def tab_display_name(self):
        if self._current_file:
            name = os.path.basename(self._current_file)
        else:
            name = "Untitled"
        if self._modified:
            name += " *"
        return name


class MotionEditor(QMainWindow):
    """UR5e 50-Pose Motion Editor GUI (탭 기반)"""

    def __init__(self):
        super().__init__()
        self.current_q = np.zeros(6)
        self._play_queue = []
        self._play_step = 0
        self._play_tab = None
        self._clipboard = []  # 탭 간 복사 버퍼

        self._play_timer = QTimer()
        self._play_timer.setSingleShot(True)
        self._play_timer.timeout.connect(self._play_next)

        self.init_ui()
        self.setWindowTitle("UR5e Motion Editor 🎛️")
        self.setGeometry(100, 100, 1100, 780)

    def init_ui(self):
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

        # 탭 위젯
        self.tab_widget = QTabWidget()
        self.tab_widget.setTabsClosable(True)
        self.tab_widget.tabCloseRequested.connect(self._close_tab)
        layout.addWidget(self.tab_widget)

        # 초기 탭 생성
        self._add_new_tab()

        # 체크박스 선택 버튼 행
        check_layout = QHBoxLayout()
        self.select_all_btn = QPushButton("☑ Select All (Saved)")
        self.select_all_btn.setStyleSheet("padding: 6px; font-size: 12px;")
        self.select_all_btn.clicked.connect(lambda: self._current_tab().select_all_poses())

        self.deselect_all_btn = QPushButton("☐ Deselect All")
        self.deselect_all_btn.setStyleSheet("padding: 6px; font-size: 12px;")
        self.deselect_all_btn.clicked.connect(lambda: self._current_tab().deselect_all_poses())

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

        self.clear_btn = QPushButton("🗑️ Delete Selected")
        self.clear_btn.setStyleSheet("background: #f44336; color: white; padding: 10px; font-size: 13px;")
        self.clear_btn.clicked.connect(self.delete_rows)

        btn_layout.addWidget(self.save_btn)
        btn_layout.addWidget(self.load_btn)
        btn_layout.addWidget(self.insert_btn)
        btn_layout.addWidget(self.play_btn)
        btn_layout.addWidget(self.stop_btn)
        btn_layout.addWidget(self.clear_btn)
        layout.addLayout(btn_layout)

        # 복사/붙여넣기 버튼 행
        copy_layout = QHBoxLayout()

        self.copy_btn = QPushButton("📋 Copy Rows")
        self.copy_btn.setStyleSheet("background: #607D8B; color: white; padding: 8px; font-size: 12px;")
        self.copy_btn.setToolTip("Copy selected rows to clipboard (for pasting into another tab)")
        self.copy_btn.clicked.connect(self.copy_rows)

        self.paste_btn = QPushButton("📌 Paste Rows")
        self.paste_btn.setStyleSheet("background: #795548; color: white; padding: 8px; font-size: 12px;")
        self.paste_btn.setToolTip("Paste copied rows after the selected row")
        self.paste_btn.clicked.connect(self.paste_rows)

        copy_layout.addWidget(self.copy_btn)
        copy_layout.addWidget(self.paste_btn)
        copy_layout.addStretch()
        layout.addLayout(copy_layout)

        # 메뉴바
        menubar = self.menuBar()
        file_menu = menubar.addMenu("File")

        new_action = file_menu.addAction("New Tab")
        new_action.triggered.connect(lambda: self._add_new_tab())

        save_action = file_menu.addAction("Save Motion to JSON")
        save_action.triggered.connect(self.save_json)

        load_action = file_menu.addAction("Load Motion from JSON")
        load_action.triggered.connect(self.load_json)

        file_menu.addSeparator()
        exit_action = file_menu.addAction("Exit")
        exit_action.triggered.connect(self.close)

    # ──────────────────────────── 탭 관리 ────────────────────────────

    def _current_tab(self):
        return self.tab_widget.currentWidget()

    def _add_new_tab(self, file_path=None):
        tab = MotionTab(self, file_path)
        idx = self.tab_widget.addTab(tab, tab.tab_display_name())
        self.tab_widget.setCurrentIndex(idx)
        return tab

    def update_tab_title(self, tab):
        idx = self.tab_widget.indexOf(tab)
        if idx >= 0:
            self.tab_widget.setTabText(idx, tab.tab_display_name())

    def _close_tab(self, index):
        tab = self.tab_widget.widget(index)
        if tab._modified:
            reply = QMessageBox.question(
                self, "Unsaved Changes",
                f"'{tab.tab_display_name().rstrip(' *')}' has unsaved changes.\n"
                "Close without saving?",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply != QMessageBox.Yes:
                return

        # 최소 1개 탭 유지
        if self.tab_widget.count() <= 1:
            QMessageBox.information(self, "Info", "Cannot close the last tab.")
            return

        self.tab_widget.removeTab(index)

    # ──────────────────────────── 포즈 조작 (현재 탭 위임) ──────────────────

    def update_joints(self, q):
        self.current_q = q.copy()
        for i, val in enumerate(q):
            self.joint_labels[i].setText(f"J{i+1}: {np.degrees(val):.2f}°")
        self.status_label.setText("🟢 Live - Ready to save")

    def save_pose(self):
        tab = self._current_tab()
        if tab is None:
            return
        row = tab.save_pose(self.current_q)
        self.status_label.setText(f"✅ Saved to Pose {row+1}")

    def load_pose(self):
        tab = self._current_tab()
        if tab is None:
            return
        row = tab.load_pose()
        if row is not None:
            self.status_label.setText(f"📤 Loading Pose {row+1}...")
            if hasattr(self, 'ros_node'):
                self.ros_node.publish_pose(tab.poses[row])

    def insert_row(self):
        tab = self._current_tab()
        if tab is None:
            return
        result = tab.insert_row()
        if result is not None:
            self.status_label.setText(f"➕ Inserted empty row after row {result+1}")

    def delete_rows(self):
        tab = self._current_tab()
        if tab is None:
            return
        count = tab.delete_rows()
        if count > 0:
            self.status_label.setText(f"🗑️ Deleted {count} row(s) — rows shifted up")

    def copy_rows(self):
        tab = self._current_tab()
        if tab is None:
            return
        self._clipboard = tab.get_selected_row_data()
        if not self._clipboard:
            QMessageBox.warning(self, "Warning", "No rows selected to copy!")
            return
        self.status_label.setText(f"📋 Copied {len(self._clipboard)} row(s) to clipboard")

    def paste_rows(self):
        tab = self._current_tab()
        if tab is None:
            return
        if not self._clipboard:
            QMessageBox.warning(self, "Warning", "Clipboard is empty!\nCopy rows first.")
            return

        selected = tab.pose_table.selectedIndexes()
        if not selected:
            QMessageBox.warning(self, "Warning",
                                "No row selected!\nSelect a row to paste after.")
            return

        position = selected[0].row()
        tab.insert_rows_at(position, self._clipboard)
        self.status_label.setText(
            f"📌 Pasted {len(self._clipboard)} row(s) after row {position+1}")

    # ──────────────────────────── 재생 ────────────────────────────

    def play_motion(self):
        tab = self._current_tab()
        if tab is None:
            return

        checked = tab._checked_rows()
        if not checked:
            QMessageBox.warning(self, "Warning",
                                "No poses checked!\nUse the checkboxes to select poses to play.")
            return

        valid = [(i, tab.poses[i]) for i in checked
                 if np.linalg.norm(tab.poses[i]) > 0.001]
        if not valid:
            QMessageBox.warning(self, "Warning", "Checked poses are all empty!")
            return

        timing_preview = "\n".join(
            f"  Pose {row+1}: traj={tab._get_row_timing(row)[0]:.1f}s + "
            f"wait={tab._get_row_timing(row)[1]:.1f}s"
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

        self._play_tab = tab
        self._play_queue = valid
        self._play_step = 0
        self.play_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self._play_next()

    def _play_next(self):
        tab = self._play_tab
        if tab is None:
            return

        if self._play_step > 0:
            prev_row = self._play_queue[self._play_step - 1][0]
            tab.set_row_highlight(prev_row, False)

        if self._play_step >= len(self._play_queue):
            self.play_btn.setEnabled(True)
            self.stop_btn.setEnabled(False)
            self.status_label.setText(f"✅ Motion complete ({len(self._play_queue)} poses)")
            return

        row, pose = self._play_queue[self._play_step]
        tab.set_row_highlight(row, True)
        tab.pose_table.scrollToItem(tab.pose_table.item(row, COL_NUM))

        traj, wait = tab._get_row_timing(row)
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
        self._play_timer.stop()
        tab = self._play_tab
        if tab and 0 < self._play_step <= len(self._play_queue):
            tab.set_row_highlight(self._play_queue[self._play_step - 1][0], False)
        self._play_queue = []
        self._play_step = 0
        self._play_tab = None
        self.play_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.status_label.setText("⏹ Stopped")

    # ──────────────────────────── JSON 저장/로드 ────────────────────────────

    def save_json(self):
        tab = self._current_tab()
        if tab is None:
            return
        filename = tab.save_json()
        if filename:
            self.status_label.setText(f"💾 Saved to {filename}")

    def load_json(self):
        filename, _ = QFileDialog.getOpenFileName(
            self, "Load Motion", "",
            "JSON Files (*.json);;All Files (*)")
        if not filename:
            return

        try:
            # 현재 탭이 비어있으면 (Untitled + 미수정) 재활용, 아니면 새 탭 생성
            tab = self._current_tab()
            if tab and tab._current_file is None and not tab._modified:
                pass  # 기존 빈 탭 재활용
            else:
                tab = self._add_new_tab()

            tab.load_from_json(filename)
            self.update_tab_title(tab)
            self.status_label.setText(f"📂 Loaded from {os.path.basename(filename)}")
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
            Float64MultiArray, '/target_joint_positions', qos)

        self.get_logger().info("Motion Editor ROS Node started")

    def joint_callback(self, msg):
        if len(msg.position) >= 6:
            q = np.array(msg.position[:6])
            self.gui.update_joints(q)

    def publish_pose(self, pose):
        msg = Float64MultiArray()
        msg.data = pose.tolist()
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Published pose: {pose}")


def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    signal.signal(signal.SIGINT, lambda *_: app.quit())

    gui = MotionEditor()
    gui.show()

    ros_node = ROSNode(gui)

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0) if rclpy.ok() else None)
    timer.start(10)

    exit_code = app.exec_()

    timer.stop()
    ros_node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
