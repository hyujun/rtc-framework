#!/usr/bin/env python3

import os
import sys
import json
import signal
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QTableWidget, QTableWidgetItem, QTabWidget,
    QGroupBox, QSpinBox, QDoubleSpinBox, QFileDialog, QMessageBox,
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QBrush, QColor, QFont

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from rtc_msgs.msg import GuiPosition

# ── 상수 ──────────────────────────────────────────────────────────────────────
NUM_JOINTS = 6
NUM_HAND_MOTORS = 10

HAND_FINGER_GROUPS = [
    ("Thumb",  ["thumb_cmc_aa", "thumb_cmc_fe", "thumb_mcp_fe"]),
    ("Index",  ["index_mcp_aa", "index_mcp_fe", "index_dip_fe"]),
    ("Middle", ["middle_mcp_aa", "middle_mcp_fe", "middle_dip_fe"]),
    ("Ring",   ["ring_mcp_fe"]),
]
HAND_MOTOR_NAMES = [m for _, motors in HAND_FINGER_GROUPS for m in motors]

# 컬럼 인덱스
COL_NUM = 0           # # (체크박스)
COL_NAME = 1          # Name
COL_STATUS = 2        # Status
COL_PREVIEW = 3       # UR5e Preview (읽기전용)
COL_HAND_PREVIEW = 4  # Hand Preview (읽기전용)
COL_TRAJ = 5          # Traj (s)
COL_WAIT = 6          # Wait (s)
COL_DESC = 7          # Description
NUM_COLS = 8

DEFAULT_POSES = 50
MIN_POSES = 1
MAX_POSES_LIMIT = 500

# ── Catppuccin Mocha 다크 테마 색상 ───────────────────────────────────────────
CAT_BASE = "#1e1e2e"
CAT_MANTLE = "#181825"
CAT_CRUST = "#11111b"
CAT_SURFACE0 = "#313244"
CAT_SURFACE1 = "#45475a"
CAT_SURFACE2 = "#585b70"
CAT_TEXT = "#cdd6f4"
CAT_SUBTEXT = "#a6adc8"
CAT_BLUE = "#89b4fa"
CAT_GREEN = "#a6e3a1"
CAT_RED = "#f38ba8"
CAT_ORANGE = "#fab387"
CAT_YELLOW = "#f9e2af"
CAT_MAUVE = "#cba6f7"
CAT_TEAL = "#94e2d5"

# 행 배경색 (다크 테마)
COLOR_NORMAL = QColor(CAT_BASE)
COLOR_MODIFIED = QColor(CAT_SURFACE1)
COLOR_ADDED = QColor(CAT_SURFACE0)
COLOR_PLAYING = QColor(CAT_SURFACE2)

# Diff 전경색
COLOR_NORMAL_FG = QColor(CAT_TEXT)
COLOR_MODIFIED_FG = QColor(CAT_YELLOW)
COLOR_ADDED_FG = QColor(CAT_GREEN)
COLOR_PLAYING_FG = QColor(CAT_ORANGE)

# ── Catppuccin Qt 스타일시트 ──────────────────────────────────────────────────
CATPPUCCIN_STYLESHEET = f"""
QMainWindow, QWidget {{
    background-color: {CAT_BASE};
    color: {CAT_TEXT};
}}
QGroupBox {{
    background-color: {CAT_SURFACE0};
    border: 1px solid {CAT_SURFACE1};
    border-radius: 6px;
    margin-top: 12px;
    padding-top: 14px;
    font-weight: bold;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 4px;
    color: {CAT_BLUE};
}}
QLabel {{
    color: {CAT_TEXT};
    background: transparent;
}}
QPushButton {{
    background-color: {CAT_SURFACE0};
    color: {CAT_TEXT};
    border: 1px solid {CAT_SURFACE1};
    border-radius: 4px;
    padding: 8px 14px;
    font-size: 13px;
    font-weight: bold;
}}
QPushButton:hover {{
    background-color: {CAT_SURFACE1};
}}
QPushButton:pressed {{
    background-color: {CAT_SURFACE2};
}}
QPushButton:disabled {{
    background-color: {CAT_MANTLE};
    color: {CAT_SURFACE2};
}}
QPushButton#saveBtn {{
    background-color: #40a02b; color: {CAT_CRUST};
}}
QPushButton#saveBtn:hover {{ background-color: {CAT_GREEN}; }}
QPushButton#loadBtn {{
    background-color: {CAT_BLUE}; color: {CAT_CRUST};
}}
QPushButton#loadBtn:hover {{ background-color: #b4d0fb; }}
QPushButton#insertBtn {{
    background-color: {CAT_MAUVE}; color: {CAT_CRUST};
}}
QPushButton#insertBtn:hover {{ background-color: #d9c2fa; }}
QPushButton#playBtn {{
    background-color: {CAT_ORANGE}; color: {CAT_CRUST};
}}
QPushButton#playBtn:hover {{ background-color: #fcc9a3; }}
QPushButton#stopBtn {{
    background-color: {CAT_SURFACE2}; color: {CAT_TEXT};
}}
QPushButton#deleteBtn {{
    background-color: {CAT_RED}; color: {CAT_CRUST};
}}
QPushButton#deleteBtn:hover {{ background-color: #f5a0b8; }}
QPushButton#copyBtn {{
    background-color: {CAT_TEAL}; color: {CAT_CRUST};
}}
QPushButton#copyBtn:hover {{ background-color: #b0ede3; }}
QPushButton#pasteBtn {{
    background-color: #8087a2; color: {CAT_CRUST};
}}
QPushButton#pasteBtn:hover {{ background-color: #9ca0b0; }}
QPushButton#selectAllBtn, QPushButton#deselectBtn {{
    background-color: {CAT_SURFACE0}; color: {CAT_SUBTEXT};
    padding: 5px 10px; font-size: 12px;
}}
QTableWidget {{
    background-color: {CAT_BASE};
    alternate-background-color: {CAT_MANTLE};
    color: {CAT_TEXT};
    gridline-color: {CAT_SURFACE1};
    border: 1px solid {CAT_SURFACE1};
    selection-background-color: {CAT_SURFACE2};
    selection-color: {CAT_TEXT};
}}
QTableWidget::item {{
    padding: 2px;
}}
QHeaderView::section {{
    background-color: {CAT_SURFACE0};
    color: {CAT_BLUE};
    border: 1px solid {CAT_SURFACE1};
    padding: 4px;
    font-weight: bold;
}}
QTabWidget::pane {{
    border: 1px solid {CAT_SURFACE1};
    background-color: {CAT_BASE};
}}
QTabBar::tab {{
    background-color: {CAT_SURFACE0};
    color: {CAT_SUBTEXT};
    border: 1px solid {CAT_SURFACE1};
    padding: 6px 14px;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
}}
QTabBar::tab:selected {{
    background-color: {CAT_BASE};
    color: {CAT_TEXT};
    border-bottom: 2px solid {CAT_BLUE};
}}
QTabBar::tab:hover {{
    background-color: {CAT_SURFACE1};
}}
QSpinBox, QDoubleSpinBox {{
    background-color: {CAT_SURFACE0};
    color: {CAT_TEXT};
    border: 1px solid {CAT_SURFACE1};
    border-radius: 3px;
    padding: 2px;
}}
QMenuBar {{
    background-color: {CAT_MANTLE};
    color: {CAT_TEXT};
}}
QMenuBar::item:selected {{
    background-color: {CAT_SURFACE1};
}}
QMenu {{
    background-color: {CAT_SURFACE0};
    color: {CAT_TEXT};
    border: 1px solid {CAT_SURFACE1};
}}
QMenu::item:selected {{
    background-color: {CAT_SURFACE1};
}}
QScrollBar:vertical {{
    background: {CAT_MANTLE};
    width: 12px;
}}
QScrollBar::handle:vertical {{
    background: {CAT_SURFACE1};
    border-radius: 4px;
    min-height: 20px;
}}
QScrollBar:horizontal {{
    background: {CAT_MANTLE};
    height: 12px;
}}
QScrollBar::handle:horizontal {{
    background: {CAT_SURFACE1};
    border-radius: 4px;
    min-width: 20px;
}}
QScrollBar::add-line, QScrollBar::sub-line {{
    height: 0; width: 0;
}}
"""


def _has_data(pose, hand_pose=None):
    if np.linalg.norm(pose) > 0.001:
        return True
    if hand_pose is not None and np.linalg.norm(hand_pose) > 0.001:
        return True
    return False


def _format_ur5e_preview(q):
    q_deg = np.degrees(q)
    return (
        f"[{q_deg[0]:.2f}, {q_deg[1]:.2f}, {q_deg[2]:.2f}, "
        f"{q_deg[3]:.2f}, {q_deg[4]:.2f}, {q_deg[5]:.2f}]"
    )


def _format_hand_preview(h):
    h_deg = np.degrees(h)
    prefixes = ["T", "I", "M", "R"]
    indices_list = [[0, 1, 2], [3, 4, 5], [6, 7, 8], [9]]
    parts = []
    for prefix, indices in zip(prefixes, indices_list):
        vals = ",".join(f"{h_deg[j]:.1f}" for j in indices)
        parts.append(f"{prefix}:{vals}")
    return " ".join(parts)


class MotionTab(QWidget):
    """단일 모션 파일을 관리하는 탭 위젯."""

    def __init__(self, parent_editor, num_poses=DEFAULT_POSES, file_path=None):
        super().__init__()
        self.parent_editor = parent_editor
        self._current_file = file_path
        self._modified = False
        self._num_poses = num_poses

        self.poses = [np.zeros(NUM_JOINTS) for _ in range(self._num_poses)]
        self.hand_poses = [np.zeros(NUM_HAND_MOTORS) for _ in range(self._num_poses)]
        self.pose_names = [f"Pose {i+1}" for i in range(self._num_poses)]
        self.pose_descriptions = ["" for _ in range(self._num_poses)]

        # 파일에서 로드된 원본 스냅샷 (diff 비교용)
        self._file_snapshot = None
        self._file_num_poses = 0

        self._init_table()

    @property
    def num_poses(self):
        return self._num_poses

    # ──────────────────────────── 스핀박스 팩토리 ────────────────────────────

    def _make_traj_spin(self, value=2.0):
        spin = QDoubleSpinBox()
        spin.setRange(0.1, 60.0)
        spin.setSingleStep(0.5)
        spin.setValue(value)
        spin.setDecimals(1)
        spin.setFrame(False)
        spin.setToolTip("Trajectory duration (s)")
        spin.valueChanged.connect(self._mark_modified)
        return spin

    def _make_wait_spin(self, value=0.0):
        spin = QDoubleSpinBox()
        spin.setRange(0.0, 60.0)
        spin.setSingleStep(0.1)
        spin.setValue(value)
        spin.setDecimals(1)
        spin.setFrame(False)
        spin.setToolTip("Wait duration (s)")
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

        # 정보 바
        info_bar = QHBoxLayout()
        info_bar.addWidget(QLabel("Rows:"))
        self.row_count_spin = QSpinBox()
        self.row_count_spin.setRange(MIN_POSES, MAX_POSES_LIMIT)
        self.row_count_spin.setValue(self._num_poses)
        self.row_count_spin.setToolTip("Total number of rows")
        self.row_count_spin.editingFinished.connect(self._on_row_count_changed)
        info_bar.addWidget(self.row_count_spin)

        info_bar.addSpacing(20)
        self.info_label = QLabel()
        self.info_label.setFont(QFont("Arial", 10))
        info_bar.addWidget(self.info_label)
        info_bar.addStretch()
        layout.addLayout(info_bar)

        # 테이블
        self.pose_table = QTableWidget(self._num_poses, NUM_COLS)
        self.pose_table.setHorizontalHeaderLabels([
            "#", "Name", "Status", "UR5e Preview",
            "Hand Preview", "Traj (s)", "Wait (s)", "Description",
        ])
        self.pose_table.setColumnWidth(COL_NUM, 50)
        self.pose_table.setColumnWidth(COL_NAME, 150)
        self.pose_table.setColumnWidth(COL_STATUS, 90)
        self.pose_table.setColumnWidth(COL_PREVIEW, 320)
        self.pose_table.setColumnWidth(COL_HAND_PREVIEW, 260)
        self.pose_table.setColumnWidth(COL_TRAJ, 75)
        self.pose_table.setColumnWidth(COL_WAIT, 75)
        self.pose_table.setColumnWidth(COL_DESC, 200)
        self.pose_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.pose_table.setSelectionMode(QTableWidget.MultiSelection)
        self.pose_table.itemChanged.connect(self._on_item_changed)

        for i in range(self._num_poses):
            self._init_row(i)

        layout.addWidget(self.pose_table)
        self._update_info_label()

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
        preview_item.setToolTip("UR5e joint positions (degrees)")
        self.pose_table.setItem(i, COL_PREVIEW, preview_item)

        hand_preview_item = QTableWidgetItem("-")
        hand_preview_item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
        hand_preview_item.setToolTip("Hand motor positions (degrees)")
        self.pose_table.setItem(i, COL_HAND_PREVIEW, hand_preview_item)

        self.pose_table.setCellWidget(i, COL_TRAJ, self._make_traj_spin(traj))
        self.pose_table.setCellWidget(i, COL_WAIT, self._make_wait_spin(wait))

        desc = self.pose_descriptions[i] if i < len(self.pose_descriptions) else ""
        self.pose_table.setItem(i, COL_DESC, QTableWidgetItem(desc))

    # ──────────────────────────── Row 수 조정 ────────────────────────────

    def _on_row_count_changed(self):
        new_count = self.row_count_spin.value()
        if new_count == self._num_poses:
            return

        if new_count < self._num_poses:
            lost = [i + 1 for i in range(new_count, self._num_poses)
                    if _has_data(self.poses[i], self.hand_poses[i])]
            if lost:
                reply = QMessageBox.question(
                    self, "Confirm",
                    f"Reducing rows will delete saved data in row(s): {lost}\n"
                    "Continue?",
                    QMessageBox.Yes | QMessageBox.No)
                if reply != QMessageBox.Yes:
                    self.row_count_spin.setValue(self._num_poses)
                    return

        self._resize_to(new_count)

    def _resize_to(self, new_count):
        old_count = self._num_poses

        traj_vals = [self._get_row_timing(i)[0] for i in range(old_count)]
        wait_vals = [self._get_row_timing(i)[1] for i in range(old_count)]

        if new_count > old_count:
            for _ in range(new_count - old_count):
                self.poses.append(np.zeros(NUM_JOINTS))
                self.hand_poses.append(np.zeros(NUM_HAND_MOTORS))
                self.pose_names.append(f"Pose {len(self.poses)}")
                self.pose_descriptions.append("")
                traj_vals.append(2.0)
                wait_vals.append(0.0)
        else:
            self.poses = self.poses[:new_count]
            self.hand_poses = self.hand_poses[:new_count]
            self.pose_names = self.pose_names[:new_count]
            self.pose_descriptions = self.pose_descriptions[:new_count]
            traj_vals = traj_vals[:new_count]
            wait_vals = wait_vals[:new_count]

        self._num_poses = new_count
        self.row_count_spin.setValue(new_count)

        self.pose_table.itemChanged.disconnect(self._on_item_changed)
        self.pose_table.setRowCount(new_count)

        if new_count > old_count:
            for i in range(old_count, new_count):
                self._init_row(i, traj_vals[i], wait_vals[i])

        self._rebuild_table(traj_vals, wait_vals, reconnect=False)
        self.pose_table.itemChanged.connect(self._on_item_changed)
        self._mark_modified()
        self._update_info_label()

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
        self._update_info_label()
        self._update_diff_highlight()

    def _mark_saved(self):
        self._modified = False
        self._take_snapshot()
        self.parent_editor.update_tab_title(self)
        self._update_info_label()
        self._update_diff_highlight()

    def _checked_rows(self):
        return [i for i in range(self._num_poses)
                if self.pose_table.item(i, COL_NUM).checkState() == Qt.Checked]

    def select_all_poses(self):
        self.pose_table.itemChanged.disconnect(self._on_item_changed)
        for i in range(self._num_poses):
            if _has_data(self.poses[i], self.hand_poses[i]):
                self.pose_table.item(i, COL_NUM).setCheckState(Qt.Checked)
        self.pose_table.itemChanged.connect(self._on_item_changed)

    def deselect_all_poses(self):
        self.pose_table.itemChanged.disconnect(self._on_item_changed)
        for i in range(self._num_poses):
            self.pose_table.item(i, COL_NUM).setCheckState(Qt.Unchecked)
        self.pose_table.itemChanged.connect(self._on_item_changed)

    # ──────────────────────────── 포즈 카운트 / Diff ────────────────────────

    def _gui_saved_count(self):
        return sum(1 for i in range(len(self.poses))
                   if _has_data(self.poses[i], self.hand_poses[i]))

    def _update_info_label(self):
        gui_count = self._gui_saved_count()
        parts = [f"GUI: {gui_count}/{self._num_poses} saved"]
        if self._file_snapshot is not None:
            parts.append(f"File: {self._file_num_poses} saved")
            diff = gui_count - self._file_num_poses
            if diff > 0:
                parts.append(f"(+{diff})")
            elif diff < 0:
                parts.append(f"({diff})")
        elif self._current_file is None:
            parts.append("(not saved to file)")
        self.info_label.setText("  |  ".join(parts))

    def _take_snapshot(self):
        self._file_snapshot = {
            'poses': [p.copy() for p in self.poses],
            'hand_poses': [p.copy() for p in self.hand_poses],
            'descriptions': list(self.pose_descriptions),
            'traj': [self._get_row_timing(i)[0] for i in range(self._num_poses)],
            'wait': [self._get_row_timing(i)[1] for i in range(self._num_poses)],
            'num_poses': self._num_poses,
        }
        self._file_num_poses = self._gui_saved_count()

    def _is_row_different(self, row):
        if self._file_snapshot is None:
            return False
        snap = self._file_snapshot
        if row >= snap['num_poses']:
            return _has_data(self.poses[row], self.hand_poses[row])
        if not np.allclose(self.poses[row], snap['poses'][row], atol=1e-6):
            return True
        if not np.allclose(self.hand_poses[row], snap['hand_poses'][row], atol=1e-6):
            return True
        if self.pose_descriptions[row] != snap['descriptions'][row]:
            return True
        traj, wait = self._get_row_timing(row)
        if abs(traj - snap['traj'][row]) > 0.01 or abs(wait - snap['wait'][row]) > 0.01:
            return True
        return False

    def _update_diff_highlight(self):
        for i in range(self._num_poses):
            if self._file_snapshot is None:
                color, fg = COLOR_NORMAL, COLOR_NORMAL_FG
            elif i >= self._file_snapshot['num_poses']:
                if _has_data(self.poses[i], self.hand_poses[i]):
                    color, fg = COLOR_ADDED, COLOR_ADDED_FG
                else:
                    color, fg = COLOR_NORMAL, COLOR_NORMAL_FG
            elif self._is_row_different(i):
                color, fg = COLOR_MODIFIED, COLOR_MODIFIED_FG
            else:
                color, fg = COLOR_NORMAL, COLOR_NORMAL_FG
            self._set_row_bg(i, color, fg)

    def _set_row_bg(self, row, color, fg=None):
        bg_brush = QBrush(color)
        fg_brush = QBrush(fg) if fg else QBrush(COLOR_NORMAL_FG)
        for col in range(self.pose_table.columnCount()):
            item = self.pose_table.item(row, col)
            if item:
                item.setBackground(bg_brush)
                item.setForeground(fg_brush)

    # ──────────────────────────── 포즈 조작 ────────────────────────────

    def save_pose(self, current_q, current_hand=None):
        selected = self.pose_table.selectedIndexes()
        if not selected:
            row = next((i for i in range(self._num_poses)
                        if not _has_data(self.poses[i], self.hand_poses[i])), 0)
        else:
            row = selected[0].row()

        self.poses[row] = current_q.copy()
        if current_hand is not None:
            self.hand_poses[row] = current_hand.copy()

        self.pose_table.item(row, COL_STATUS).setText("Saved")

        preview = self.pose_table.item(row, COL_PREVIEW)
        preview.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
        preview.setText(_format_ur5e_preview(current_q))

        hand_preview = self.pose_table.item(row, COL_HAND_PREVIEW)
        hand_preview.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
        if current_hand is not None and np.linalg.norm(current_hand) > 0.001:
            hand_preview.setText(_format_hand_preview(current_hand))
        else:
            hand_preview.setText("-")

        self.pose_table.clearSelection()
        self._mark_modified()
        return row

    def load_pose(self):
        selected = self.pose_table.selectedIndexes()
        if not selected:
            QMessageBox.warning(self, "Warning", "No pose selected!")
            return None
        row = selected[0].row()
        if not _has_data(self.poses[row], self.hand_poses[row]):
            QMessageBox.warning(self, "Warning", f"Pose {row+1} is empty!")
            return None
        return row

    def insert_row(self):
        selected = self.pose_table.selectedIndexes()
        if not selected:
            QMessageBox.warning(
                self, "Warning",
                "No row selected!\nClick a row to select it first.")
            return None

        insert_after = selected[0].row()
        ins = insert_after + 1

        last = self._num_poses - 1
        if _has_data(self.poses[last], self.hand_poses[last]):
            reply = QMessageBox.question(
                self, "Confirm",
                f"Inserting a row will drop Pose {last+1} "
                "(last row has saved data).\nContinue?",
                QMessageBox.Yes | QMessageBox.No)
            if reply != QMessageBox.Yes:
                return None

        traj_vals = [self._get_row_timing(i)[0] for i in range(self._num_poses)]
        wait_vals = [self._get_row_timing(i)[1] for i in range(self._num_poses)]

        self.poses.insert(ins, np.zeros(NUM_JOINTS))
        self.poses.pop()
        self.hand_poses.insert(ins, np.zeros(NUM_HAND_MOTORS))
        self.hand_poses.pop()
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
        n = len(row_data_list)
        if n == 0:
            return

        traj_vals = [self._get_row_timing(i)[0] for i in range(self._num_poses)]
        wait_vals = [self._get_row_timing(i)[1] for i in range(self._num_poses)]

        lost_rows = []
        for i in range(self._num_poses - n, self._num_poses):
            if i >= 0 and _has_data(self.poses[i], self.hand_poses[i]):
                lost_rows.append(i + 1)
        if lost_rows:
            reply = QMessageBox.question(
                self, "Confirm",
                f"Inserting {n} row(s) will drop data in row(s): "
                f"{lost_rows}.\nContinue?",
                QMessageBox.Yes | QMessageBox.No)
            if reply != QMessageBox.Yes:
                return

        ins = position + 1
        for j, rd in enumerate(row_data_list):
            idx = ins + j
            self.poses.insert(idx, rd['pose'].copy())
            self.poses.pop()
            hp = rd.get('hand_pose', np.zeros(NUM_HAND_MOTORS))
            self.hand_poses.insert(idx, hp.copy())
            self.hand_poses.pop()
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
        selected = self.pose_table.selectedIndexes()
        if not selected:
            return 0

        rows = sorted({idx.row() for idx in selected}, reverse=True)

        traj_vals = [self._get_row_timing(i)[0] for i in range(self._num_poses)]
        wait_vals = [self._get_row_timing(i)[1] for i in range(self._num_poses)]

        for row in rows:
            self.poses.pop(row)
            self.hand_poses.pop(row)
            self.pose_names.pop(row)
            self.pose_descriptions.pop(row)
            traj_vals.pop(row)
            wait_vals.pop(row)

        deleted_count = len(rows)
        while len(self.poses) < self._num_poses:
            idx = len(self.poses)
            self.poses.append(np.zeros(NUM_JOINTS))
            self.hand_poses.append(np.zeros(NUM_HAND_MOTORS))
            self.pose_names.append(f"Pose {idx+1}")
            self.pose_descriptions.append("")
            traj_vals.append(2.0)
            wait_vals.append(0.0)

        self._rebuild_table(traj_vals, wait_vals)
        self.pose_table.clearSelection()
        self._mark_modified()
        return deleted_count

    def get_selected_row_data(self):
        selected = self.pose_table.selectedIndexes()
        if not selected:
            return []

        rows = sorted({idx.row() for idx in selected})
        data = []
        for row in rows:
            traj, wait = self._get_row_timing(row)
            data.append({
                'pose': self.poses[row].copy(),
                'hand_pose': self.hand_poses[row].copy(),
                'name': self.pose_names[row],
                'description': self.pose_descriptions[row],
                'traj': traj,
                'wait': wait,
            })
        return data

    def _rebuild_table(self, traj_vals=None, wait_vals=None, reconnect=True):
        if traj_vals is None:
            traj_vals = [self._get_row_timing(i)[0] for i in range(self._num_poses)]
        if wait_vals is None:
            wait_vals = [self._get_row_timing(i)[1] for i in range(self._num_poses)]

        if reconnect:
            self.pose_table.itemChanged.disconnect(self._on_item_changed)

        for i in range(self._num_poses):
            has_pose = _has_data(self.poses[i], self.hand_poses[i])

            num_item = self.pose_table.item(i, COL_NUM)
            num_item.setText(str(i + 1))
            num_item.setCheckState(Qt.Unchecked)

            self.pose_names[i] = f"Pose {i+1}"
            self.pose_table.item(i, COL_NAME).setText(self.pose_names[i])

            self.pose_table.item(i, COL_STATUS).setText(
                "Saved" if has_pose else "Empty")

            # UR5e preview
            preview = self.pose_table.item(i, COL_PREVIEW)
            preview.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
            if np.linalg.norm(self.poses[i]) > 0.001:
                preview.setText(_format_ur5e_preview(self.poses[i]))
            else:
                preview.setText("-")

            # Hand preview
            hand_preview = self.pose_table.item(i, COL_HAND_PREVIEW)
            hand_preview.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
            if np.linalg.norm(self.hand_poses[i]) > 0.001:
                hand_preview.setText(_format_hand_preview(self.hand_poses[i]))
            else:
                hand_preview.setText("-")

            # Traj / Wait
            traj_spin = self.pose_table.cellWidget(i, COL_TRAJ)
            wait_spin = self.pose_table.cellWidget(i, COL_WAIT)
            if traj_spin:
                traj_spin.setValue(traj_vals[i])
            if wait_spin:
                wait_spin.setValue(wait_vals[i])

            # Description
            self.pose_table.item(i, COL_DESC).setText(
                self.pose_descriptions[i])

        if reconnect:
            self.pose_table.itemChanged.connect(self._on_item_changed)

    def set_row_highlight(self, row, active):
        if active:
            self._set_row_bg(row, COLOR_PLAYING, COLOR_PLAYING_FG)
        else:
            self._restore_row_bg(row)

    def _restore_row_bg(self, row):
        if self._file_snapshot is None:
            color, fg = COLOR_NORMAL, COLOR_NORMAL_FG
        elif row >= self._file_snapshot['num_poses']:
            if _has_data(self.poses[row], self.hand_poses[row]):
                color, fg = COLOR_ADDED, COLOR_ADDED_FG
            else:
                color, fg = COLOR_NORMAL, COLOR_NORMAL_FG
        elif self._is_row_different(row):
            color, fg = COLOR_MODIFIED, COLOR_MODIFIED_FG
        else:
            color, fg = COLOR_NORMAL, COLOR_NORMAL_FG
        self._set_row_bg(row, color, fg)

    # ──────────────────────────── JSON 저장/로드 ────────────────────────────

    def save_json(self, filename=None):
        if filename is None:
            default_name = self._current_file or ""
            if not default_name:
                session = os.environ.get('UR5E_SESSION_DIR', '')
                if session:
                    motions_dir = os.path.join(session, 'motions')
                    os.makedirs(motions_dir, exist_ok=True)
                    default_name = motions_dir
            filename, _ = QFileDialog.getSaveFileName(
                self, "Save Motion", default_name,
                "JSON Files (*.json);;All Files (*)")
            if not filename:
                return None

        if not os.path.splitext(filename)[1]:
            filename += ".json"

        traj_times, wait_times = [], []
        for i in range(self._num_poses):
            t, w = self._get_row_timing(i)
            traj_times.append(t)
            wait_times.append(w)

        data = {
            "num_poses": self._num_poses,
            "poses": {
                f"pose_{i}": self.poses[i].tolist()
                for i in range(self._num_poses)
            },
            "hand_poses": {
                f"pose_{i}": self.hand_poses[i].tolist()
                for i in range(self._num_poses)
            },
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

        file_num_poses = data.get('num_poses', len(data.get('poses', {})))
        descriptions = data.get('descriptions', [""] * file_num_poses)
        traj_times = data.get('traj_times', [2.0] * file_num_poses)
        wait_times = data.get('wait_times', [0.0] * file_num_poses)
        hand_poses_data = data.get('hand_poses', {})

        if file_num_poses != self._num_poses:
            self._resize_to(file_num_poses)

        self.pose_table.itemChanged.disconnect(self._on_item_changed)

        for i in range(min(self._num_poses, len(data.get('poses', {})))):
            key = f"pose_{i}"
            if key not in data['poses']:
                continue

            self.poses[i] = np.array(data['poses'][key])

            # 핸드 데이터 로드 (하위 호환: 없으면 zeros)
            if key in hand_poses_data:
                self.hand_poses[i] = np.array(hand_poses_data[key])
            else:
                self.hand_poses[i] = np.zeros(NUM_HAND_MOTORS)

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
                self.pose_table.item(i, COL_STATUS).setText("Saved")
                preview = self.pose_table.item(i, COL_PREVIEW)
                preview.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
                preview.setText(_format_ur5e_preview(self.poses[i]))

            if np.linalg.norm(self.hand_poses[i]) > 0.001:
                hp = self.pose_table.item(i, COL_HAND_PREVIEW)
                hp.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
                hp.setText(_format_hand_preview(self.hand_poses[i]))

        self.pose_table.itemChanged.connect(self._on_item_changed)
        self._current_file = filename
        self._modified = False
        self._take_snapshot()
        self._update_info_label()
        self._update_diff_highlight()

    def tab_display_name(self):
        if self._current_file:
            name = os.path.basename(self._current_file)
        else:
            name = "Untitled"
        if self._modified:
            name += " *"
        return name


class MotionEditor(QMainWindow):
    """UR5e Motion Editor GUI."""

    def __init__(self):
        super().__init__()
        self.current_q = np.zeros(NUM_JOINTS)
        self.current_hand = np.zeros(NUM_HAND_MOTORS)
        self._play_queue = []
        self._play_step = 0
        self._play_tab = None
        self._clipboard = []

        self._play_timer = QTimer()
        self._play_timer.setSingleShot(True)
        self._play_timer.timeout.connect(self._play_next)

        self.init_ui()
        self.setWindowTitle("UR5e Motion Editor")
        self.setGeometry(100, 100, 1400, 900)

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # ── 헤더: 상태 + E-STOP ───────────────────────────────────────
        header_layout = QHBoxLayout()
        self.status_label = QLabel("Waiting for robot...")
        self.status_label.setFont(QFont("Arial", 14, QFont.Bold))
        self.status_label.setStyleSheet(
            f"padding: 10px; background: {CAT_SURFACE0}; "
            f"border-radius: 5px; color: {CAT_TEXT};")
        header_layout.addWidget(self.status_label, stretch=1)

        header_layout.addSpacing(10)
        self.estop_badge = QLabel("  NORMAL  ")
        self.estop_badge.setFont(QFont("Arial", 11, QFont.Bold))
        self.estop_badge.setStyleSheet(
            f"background: {CAT_GREEN}; color: {CAT_CRUST}; "
            "font-weight: bold; padding: 6px 12px; border-radius: 4px;")
        header_layout.addWidget(self.estop_badge)
        layout.addLayout(header_layout)

        # ── 현재 상태: 3-컬럼 (Joint | Task | Hand) ──────────────────
        state_group = QGroupBox("Current State")
        state_layout = QHBoxLayout()

        # 왼쪽: Joint Angles
        joint_frame = QVBoxLayout()
        joint_title = QLabel("Joint Angles")
        joint_title.setFont(QFont("Arial", 11, QFont.Bold))
        joint_title.setStyleSheet(f"color: {CAT_BLUE};")
        joint_frame.addWidget(joint_title)
        self.joint_labels = []
        for i in range(NUM_JOINTS):
            label = QLabel(f"J{i+1}: 0.000")
            label.setFont(QFont("Courier", 11))
            self.joint_labels.append(label)
            joint_frame.addWidget(label)
        state_layout.addLayout(joint_frame)

        # 세로 구분선
        sep1 = QLabel("")
        sep1.setFixedWidth(2)
        sep1.setStyleSheet(f"background: {CAT_SURFACE1};")
        state_layout.addWidget(sep1)

        # 가운데: Task Position
        task_frame = QVBoxLayout()
        task_title = QLabel("Task Position")
        task_title.setFont(QFont("Arial", 11, QFont.Bold))
        task_title.setStyleSheet(f"color: {CAT_MAUVE};")
        task_frame.addWidget(task_title)
        self.task_labels = []
        task_names = ["X (m)", "Y (m)", "Z (m)", "Roll", "Pitch", "Yaw"]
        for name in task_names:
            label = QLabel(f"{name}: 0.0000")
            label.setFont(QFont("Courier", 11))
            self.task_labels.append(label)
            task_frame.addWidget(label)
        state_layout.addLayout(task_frame)

        # 세로 구분선
        sep2 = QLabel("")
        sep2.setFixedWidth(2)
        sep2.setStyleSheet(f"background: {CAT_SURFACE1};")
        state_layout.addWidget(sep2)

        # 오른쪽: Hand State
        hand_frame = QVBoxLayout()
        hand_title = QLabel("Hand State")
        hand_title.setFont(QFont("Arial", 11, QFont.Bold))
        hand_title.setStyleSheet(f"color: {CAT_TEAL};")
        hand_frame.addWidget(hand_title)
        self.hand_state_labels = []
        for finger_name, motors in HAND_FINGER_GROUPS:
            finger_label = QLabel(finger_name)
            finger_label.setFont(QFont("Arial", 9, QFont.Bold))
            finger_label.setStyleSheet(f"color: {CAT_YELLOW};")
            hand_frame.addWidget(finger_label)
            for motor in motors:
                short = motor.split('_', 1)[1] if '_' in motor else motor
                label = QLabel(f"  {short}: 0.00")
                label.setFont(QFont("Courier", 10))
                self.hand_state_labels.append(label)
                hand_frame.addWidget(label)
        state_layout.addLayout(hand_frame)

        state_group.setLayout(state_layout)
        layout.addWidget(state_group)

        # ── 범례 ─────────────────────────────────────────────────────
        legend_layout = QHBoxLayout()
        legend_items = [
            (COLOR_MODIFIED, COLOR_MODIFIED_FG, "Modified vs file"),
            (COLOR_ADDED, COLOR_ADDED_FG, "New (not in file)"),
        ]
        for bg_color, fg_color, text in legend_items:
            swatch = QLabel("  ")
            swatch.setFixedSize(16, 16)
            swatch.setStyleSheet(
                f"background: {bg_color.name()}; "
                f"border: 1px solid {CAT_SURFACE2};")
            legend_layout.addWidget(swatch)
            legend_label = QLabel(text)
            legend_label.setStyleSheet(f"color: {fg_color.name()};")
            legend_layout.addWidget(legend_label)
            legend_layout.addSpacing(15)
        legend_layout.addStretch()
        layout.addLayout(legend_layout)

        # ── 탭 위젯 ──────────────────────────────────────────────────
        self.tab_widget = QTabWidget()
        self.tab_widget.setTabsClosable(True)
        self.tab_widget.tabCloseRequested.connect(self._close_tab)
        layout.addWidget(self.tab_widget)

        self._add_new_tab()

        # ── 체크박스 선택 버튼 ────────────────────────────────────────
        check_layout = QHBoxLayout()
        self.select_all_btn = QPushButton("Select All (Saved)")
        self.select_all_btn.setObjectName("selectAllBtn")
        self.select_all_btn.clicked.connect(
            lambda: self._current_tab().select_all_poses())

        self.deselect_all_btn = QPushButton("Deselect All")
        self.deselect_all_btn.setObjectName("deselectBtn")
        self.deselect_all_btn.clicked.connect(
            lambda: self._current_tab().deselect_all_poses())

        check_layout.addWidget(self.select_all_btn)
        check_layout.addWidget(self.deselect_all_btn)
        check_layout.addStretch()
        layout.addLayout(check_layout)

        # ── 메인 버튼 ────────────────────────────────────────────────
        btn_layout = QHBoxLayout()

        self.save_btn = QPushButton("Save Current Pose")
        self.save_btn.setObjectName("saveBtn")
        self.save_btn.clicked.connect(self.save_pose)

        self.load_btn = QPushButton("Load Selected Pose")
        self.load_btn.setObjectName("loadBtn")
        self.load_btn.clicked.connect(self.load_pose)

        self.insert_btn = QPushButton("Insert Empty Row")
        self.insert_btn.setObjectName("insertBtn")
        self.insert_btn.setToolTip(
            "Insert an empty row after the selected row.\n"
            "All rows below shift down; the last row is dropped.")
        self.insert_btn.clicked.connect(self.insert_row)

        self.play_btn = QPushButton("Play Checked Poses")
        self.play_btn.setObjectName("playBtn")
        self.play_btn.clicked.connect(self.play_motion)

        self.stop_btn = QPushButton("Stop")
        self.stop_btn.setObjectName("stopBtn")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.stop_motion)

        self.clear_btn = QPushButton("Delete Selected")
        self.clear_btn.setObjectName("deleteBtn")
        self.clear_btn.clicked.connect(self.delete_rows)

        btn_layout.addWidget(self.save_btn)
        btn_layout.addWidget(self.load_btn)
        btn_layout.addWidget(self.insert_btn)
        btn_layout.addWidget(self.play_btn)
        btn_layout.addWidget(self.stop_btn)
        btn_layout.addWidget(self.clear_btn)
        layout.addLayout(btn_layout)

        # ── 복사/붙여넣기 버튼 ────────────────────────────────────────
        copy_layout = QHBoxLayout()

        self.copy_btn = QPushButton("Copy Rows")
        self.copy_btn.setObjectName("copyBtn")
        self.copy_btn.setToolTip("Copy selected rows to clipboard")
        self.copy_btn.clicked.connect(self.copy_rows)

        self.paste_btn = QPushButton("Paste Rows")
        self.paste_btn.setObjectName("pasteBtn")
        self.paste_btn.setToolTip("Paste copied rows after the selected row")
        self.paste_btn.clicked.connect(self.paste_rows)

        copy_layout.addWidget(self.copy_btn)
        copy_layout.addWidget(self.paste_btn)
        copy_layout.addStretch()
        layout.addLayout(copy_layout)

        # ── 메뉴바 ───────────────────────────────────────────────────
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

    def _add_new_tab(self, file_path=None, num_poses=DEFAULT_POSES):
        tab = MotionTab(self, num_poses=num_poses, file_path=file_path)
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
                f"'{tab.tab_display_name().rstrip(' *')}' has unsaved "
                "changes.\nClose without saving?",
                QMessageBox.Yes | QMessageBox.No)
            if reply != QMessageBox.Yes:
                return

        if self.tab_widget.count() <= 1:
            QMessageBox.information(
                self, "Info", "Cannot close the last tab.")
            return

        self.tab_widget.removeTab(index)

    # ──────────────────────────── 상태 업데이트 ──────────────────────

    def update_joints(self, q):
        self.current_q = q.copy()
        for i, val in enumerate(q):
            self.joint_labels[i].setText(
                f"J{i+1}: {np.degrees(val):.2f}")
        self.status_label.setText("Live - Ready to save")

    def update_task_position(self, task_pos):
        names_pos = ["X", "Y", "Z"]
        names_rot = ["Roll", "Pitch", "Yaw"]
        for i, val in enumerate(task_pos):
            if i < 3:
                self.task_labels[i].setText(
                    f"{names_pos[i]}: {val:.4f} m")
            else:
                self.task_labels[i].setText(
                    f"{names_rot[i-3]}: {np.degrees(val):.2f}")

    def update_hand_state(self, hand_pos):
        self.current_hand = hand_pos.copy()
        for i, val in enumerate(hand_pos):
            if i < len(self.hand_state_labels):
                motor = HAND_MOTOR_NAMES[i]
                short = motor.split('_', 1)[1] if '_' in motor else motor
                self.hand_state_labels[i].setText(
                    f"  {short}: {np.degrees(val):.2f}")

    def update_estop(self, active):
        if active:
            self.estop_badge.setText("  E-STOP ACTIVE  ")
            self.estop_badge.setStyleSheet(
                f"background: {CAT_RED}; color: {CAT_CRUST}; "
                "font-weight: bold; padding: 6px 12px; "
                "border-radius: 4px;")
        else:
            self.estop_badge.setText("  NORMAL  ")
            self.estop_badge.setStyleSheet(
                f"background: {CAT_GREEN}; color: {CAT_CRUST}; "
                "font-weight: bold; padding: 6px 12px; "
                "border-radius: 4px;")

    # ──────────────────────────── 포즈 조작 ──────────────────────────

    def save_pose(self):
        tab = self._current_tab()
        if tab is None:
            return
        row = tab.save_pose(self.current_q, self.current_hand)
        self.status_label.setText(f"Saved to Pose {row+1}")

    def load_pose(self):
        tab = self._current_tab()
        if tab is None:
            return
        row = tab.load_pose()
        if row is not None:
            self.status_label.setText(f"Loading Pose {row+1}...")
            if hasattr(self, 'ros_node'):
                self.ros_node.publish_pose(tab.poses[row])
                self.ros_node.publish_hand_pose(tab.hand_poses[row])

    def insert_row(self):
        tab = self._current_tab()
        if tab is None:
            return
        result = tab.insert_row()
        if result is not None:
            self.status_label.setText(
                f"Inserted empty row after row {result+1}")

    def delete_rows(self):
        tab = self._current_tab()
        if tab is None:
            return
        count = tab.delete_rows()
        if count > 0:
            self.status_label.setText(
                f"Deleted {count} row(s)")

    def copy_rows(self):
        tab = self._current_tab()
        if tab is None:
            return
        self._clipboard = tab.get_selected_row_data()
        if not self._clipboard:
            QMessageBox.warning(
                self, "Warning", "No rows selected to copy!")
            return
        self.status_label.setText(
            f"Copied {len(self._clipboard)} row(s) to clipboard")

    def paste_rows(self):
        tab = self._current_tab()
        if tab is None:
            return
        if not self._clipboard:
            QMessageBox.warning(
                self, "Warning",
                "Clipboard is empty!\nCopy rows first.")
            return

        selected = tab.pose_table.selectedIndexes()
        if not selected:
            QMessageBox.warning(
                self, "Warning",
                "No row selected!\nSelect a row to paste after.")
            return

        position = selected[0].row()
        tab.insert_rows_at(position, self._clipboard)
        self.status_label.setText(
            f"Pasted {len(self._clipboard)} row(s) after row {position+1}")

    # ──────────────────────────── 재생 ────────────────────────────

    def play_motion(self):
        tab = self._current_tab()
        if tab is None:
            return

        checked = tab._checked_rows()
        if not checked:
            QMessageBox.warning(
                self, "Warning",
                "No poses checked!\n"
                "Use the checkboxes to select poses to play.")
            return

        valid = [(i, tab.poses[i]) for i in checked
                 if _has_data(tab.poses[i], tab.hand_poses[i])]
        if not valid:
            QMessageBox.warning(
                self, "Warning", "Checked poses are all empty!")
            return

        timing_preview = "\n".join(
            f"  Pose {row+1}: traj={tab._get_row_timing(row)[0]:.1f}s"
            f" + wait={tab._get_row_timing(row)[1]:.1f}s"
            for row, _ in valid
        )
        reply = QMessageBox.question(
            self, "Confirm",
            f"Play {len(valid)} checked poses in sequence?\n"
            f"Poses: {[r+1 for r, _ in valid]}\n\n{timing_preview}",
            QMessageBox.Yes | QMessageBox.No)
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
            self.status_label.setText(
                f"Motion complete ({len(self._play_queue)} poses)")
            return

        row, pose = self._play_queue[self._play_step]
        tab.set_row_highlight(row, True)
        tab.pose_table.scrollToItem(
            tab.pose_table.item(row, COL_NUM))

        traj, wait = tab._get_row_timing(row)
        delay_ms = max(100, int((traj + wait) * 1000))
        self.status_label.setText(
            f"Playing Pose {row+1}  "
            f"({self._play_step+1}/{len(self._play_queue)})  "
            f"[traj={traj:.1f}s + wait={wait:.1f}s]")

        if hasattr(self, 'ros_node'):
            self.ros_node.publish_pose(pose)
            self.ros_node.publish_hand_pose(tab.hand_poses[row])

        self._play_step += 1
        self._play_timer.start(delay_ms)

    def stop_motion(self):
        self._play_timer.stop()
        tab = self._play_tab
        if tab and 0 < self._play_step <= len(self._play_queue):
            tab.set_row_highlight(
                self._play_queue[self._play_step - 1][0], False)
        self._play_queue = []
        self._play_step = 0
        self._play_tab = None
        self.play_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.status_label.setText("Stopped")

    # ──────────────────────────── JSON 저장/로드 ────────────────────────

    def save_json(self):
        tab = self._current_tab()
        if tab is None:
            return
        filename = tab.save_json()
        if filename:
            self.status_label.setText(f"Saved to {filename}")

    def load_json(self):
        filename, _ = QFileDialog.getOpenFileName(
            self, "Load Motion", "",
            "JSON Files (*.json);;All Files (*)")
        if not filename:
            return

        try:
            tab = self._current_tab()
            if tab and tab._current_file is None and not tab._modified:
                pass
            else:
                tab = self._add_new_tab()

            tab.load_from_json(filename)
            self.update_tab_title(tab)
            self.status_label.setText(
                f"Loaded from {os.path.basename(filename)}")
        except Exception as e:
            QMessageBox.critical(
                self, "Error", f"Failed to load: {str(e)}")


class ROSNode(Node):
    """ROS2 통신 노드."""

    def __init__(self, gui):
        super().__init__('motion_editor_node')
        self.gui = gui
        gui.ros_node = self

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # ── Subscribers ───────────────────────────────────────────────
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, qos)

        self.hand_sub = self.create_subscription(
            JointState, '/hand/joint_states',
            self.hand_callback, qos)

        self.task_pos_sub = self.create_subscription(
            GuiPosition, '/ur5e/gui_position',
            self.gui_pos_callback, qos)

        self.estop_sub = self.create_subscription(
            Bool, '/system/estop_status', self.estop_callback, qos)

        # ── Publishers ────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/ur5e/target_joint_positions', qos)

        self.hand_cmd_pub = self.create_publisher(
            Float64MultiArray, '/hand/target_joint_positions', qos)

        self.get_logger().info("Motion Editor ROS Node started")

    def joint_callback(self, msg):
        if len(msg.position) >= NUM_JOINTS:
            q = np.array(msg.position[:NUM_JOINTS])
            self.gui.update_joints(q)

    def hand_callback(self, msg):
        if len(msg.position) >= NUM_HAND_MOTORS:
            h = np.array(msg.position[:NUM_HAND_MOTORS])
            self.gui.update_hand_state(h)

    def gui_pos_callback(self, msg: GuiPosition):
        if len(msg.task_positions) >= 6:
            self.gui.update_task_position(np.array(msg.task_positions[:6]))

    def estop_callback(self, msg):
        self.gui.update_estop(msg.data)

    def publish_pose(self, pose):
        msg = Float64MultiArray()
        msg.data = pose.tolist()
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Published UR5e pose: {pose}")

    def publish_hand_pose(self, hand_pose):
        if np.linalg.norm(hand_pose) < 0.001:
            return
        msg = Float64MultiArray()
        msg.data = hand_pose.tolist()
        self.hand_cmd_pub.publish(msg)
        self.get_logger().info(f"Published hand pose: {hand_pose}")


def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setStyleSheet(CATPPUCCIN_STYLESHEET)
    signal.signal(signal.SIGINT, lambda *_: app.quit())

    gui = MotionEditor()
    gui.show()

    ros_node = ROSNode(gui)

    timer = QTimer()
    timer.timeout.connect(
        lambda: rclpy.spin_once(ros_node, timeout_sec=0)
        if rclpy.ok() else None)
    timer.start(10)

    exit_code = app.exec_()

    timer.stop()
    ros_node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
