#!/usr/bin/python3
# SPDX-License-Identifier: BSD-3-Clause
"""Optional, standalone ROS 2 remote. No dependency from the control core."""
import argparse
import csv
import glob
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from ehr_message.msg import LocoVelocity
from ehr_message.srv import LocomotionModeSwitch
from std_msgs.msg import String
from rclpy.validate_namespace import validate_namespace
try:
    from ehr_message.srv import MotionCmd
except ImportError:
    MotionCmd = None
from PyQt5 import QtCore, QtGui, QtWidgets
import yaml
from xbox import Xbox

MODE_NAMES = {1: "STILL · 阻尼", 2: "READY · 到位", 28: "AMP WALK",
              30: "MIMIC A", 31: "MIMIC B", 32: "MIMIC C", 33: "MIMIC D",
              34: "MIMIC E", 35: "仰面躺下", 36: "俯卧趴下",
              37: "仰卧起身", 38: "俯卧起身", 41: "感知行走",
              42: "站立下蹲", 43: "蹲姿起立"}
VELOCITY_MODES = (28, 41)
# Match eir_ctrl_demo_eqr11's policy_spec.cpp; body velocity commands use SI units.
EQR_MODE_NAMES = {1: "STILL · 阻尼", 2: "READY · 到位", 103: "TROT · 对角小跑",
                  104: "BOUND · 跳步", 105: "PACE · 同侧步态",
                  108: "FLIP", 109: "STAIR · 楼梯"}
EQR_VELOCITY_LIMITS = {103: (1.5, 0.6, 1.2), 104: (3.0, 0.5, 1.0),
                       105: (3.0, 0.6, 1.0), 108: (0.1, 0.1, 0.1),
                       109: (1.0, 1.0, 1.0)}


def load_settings(config_path, gui_path, namespace=None):
    config_path = Path(config_path).resolve()
    with config_path.open() as stream:
        config = yaml.safe_load(stream)
    with Path(gui_path).open() as stream:
        settings = yaml.safe_load(stream)
    quadruped = config.get("robot") == "eqr11"
    if config.get("robot") == "eqr11w":
        raise ValueError("EQR11W 轮足 RL 尚未接入遥控 GUI")
    settings["quadruped"] = quadruped
    settings["mode_names"] = EQR_MODE_NAMES if quadruped else MODE_NAMES
    settings["velocity_modes"] = tuple(EQR_VELOCITY_LIMITS) if quadruped else VELOCITY_MODES
    settings["robot_identity"] = ""
    if quadruped:
        ns = (namespace if namespace is not None else "/eqr11").rstrip("/") or "/"
        validate_namespace(ns)
        prefix = ns.rstrip("/")
        settings.update(modes=list(EQR_MODE_NAMES),
                        low=[-v for v in EQR_VELOCITY_LIMITS[103]],
                        high=list(EQR_VELOCITY_LIMITS[103]), velocity_timeout=1.0,
                        remote_source="external", topic=prefix + "/loco_velocity_cmd",
                        service=prefix + "/mode_switch", status_topic=prefix + "/status",
                        offline_actions=[], open_depth_viewer=False)
    else:
        if namespace is not None:
            raise ValueError("--namespace 仅用于 EQR；AIQ 使用控制器 remote 配置")
        load_aiq_controller(config_path, config, settings)
    timeout = settings["velocity_timeout"]
    for key in ("publish_hz", "keep_time", "query_period", "status_timeout",
                "service_timeout", "velocity_step", "zero_stop_seconds"):
        settings[key] = float(settings[key])
        if not math.isfinite(settings[key]) or settings[key] <= 0:
            raise ValueError(f"{key} 必须为有限正数")
    period = 1 / settings["publish_hz"]
    if not math.isfinite(timeout) or not 2 * period < settings["keep_time"] <= timeout:
        raise ValueError("keep_time 必须大于两个发布周期，且不超过控制器 velocity_timeout")
    if not settings["query_period"] < settings["status_timeout"] <= settings["service_timeout"]:
        raise ValueError("需要 query_period < status_timeout <= service_timeout")
    if not isinstance(settings["auto_center"], bool):
        raise ValueError("auto_center 必须为布尔值")
    settings["velocity_deadband"] = float(settings["velocity_deadband"])
    if not 0 <= settings["velocity_deadband"] < 1:
        raise ValueError("velocity_deadband 必须在 [0,1) 内")
    if not isinstance(settings["open_depth_viewer"], bool):
        raise ValueError("open_depth_viewer 必须为布尔值")
    for name in ("depth_image_topic", "external_depth_image_topic"):
        if not isinstance(settings[name], str):
            raise ValueError(f"{name} 必须为话题字符串")
    depth_topic = settings["depth_image_topic"] or config.get("debug", {}).get("depth_image_topic", "")
    if not depth_topic and config.get("runtime", {}).get("external_depth"):
        depth_topic = settings["external_depth_image_topic"]
    settings["depth_image_topic"] = depth_topic
    devices = settings.setdefault("remote_device_globs", ["/dev/input/js*"])
    if not isinstance(devices, list) or any(not isinstance(p, str) or not p.startswith("/") for p in devices):
        raise ValueError("remote_device_globs 必须是绝对设备路径列表")
    settings.setdefault("xbox_enabled", True)
    if not isinstance(settings["xbox_enabled"], bool):
        raise ValueError("xbox_enabled 必须为布尔值")
    nodes = settings.setdefault("remote_publisher_nodes", ["/joy_stick_publisher", "/eir_ctrl_demo_aiq11"])
    if not isinstance(nodes, list) or any(not isinstance(n, str) or not n.startswith("/") for n in nodes):
        raise ValueError("remote_publisher_nodes 必须是完整 ROS 节点名称列表")
    settings.setdefault("motion_cmd_service", "/motion_cmd")
    settings.setdefault("offline_actions", [])
    settings["motion_mode"] = None
    if not quadruped and config.get("motion_stream"):
        with (config_path.parent / config["motion_stream"]).open() as stream:
            motion = yaml.safe_load(stream)
        settings["motion_cmd_service"] = motion["service"]
        settings["offline_actions"] = [
            {"name": f"JOF{action['cmd']} · {action['name']}", "cmd": action["cmd"]}
            for action in motion["motions"]]
        settings["motion_mode"] = 28
        if config.get("runtime", {}).get("backend") == "simulation":
            settings["simulation_motion_config"] = str((config_path.parent / config["motion_stream"]).resolve())
    if not isinstance(settings["motion_cmd_service"], str) or not settings["motion_cmd_service"].strip():
        raise ValueError("motion_cmd_service 不能为空")
    actions = settings["offline_actions"]
    if not isinstance(actions, list):
        raise ValueError("offline_actions 必须是动作列表")
    ids = set()
    for action in actions:
        if (not isinstance(action, dict) or not isinstance(action.get("name"), str)
                or not action["name"].strip() or type(action.get("cmd")) is not int
                or not 11 <= action["cmd"] <= 255 or action["cmd"] in ids):
            raise ValueError("离线动作需要名称和唯一的 11～255 整数编号")
        ids.add(action["cmd"])
    return settings


def load_aiq_controller(config_path, config, settings):
    if config.get("robot"):
        with (config_path.parent / config["robot"]).open() as stream:
            robot = yaml.safe_load(stream)
        identity = robot.get("identity")
        if not isinstance(identity, str) or not identity.strip():
            raise ValueError("机器人配置缺少 identity")
        settings["robot_identity"] = identity.upper()
    if config["remote"]["source"] not in ("external", "sbus"):
        raise ValueError("remote.source 必须为 external 或 sbus")
    modes = [1, 2] + sorted(int(mode) for mode in config["policies"])
    if not any(mode in modes for mode in VELOCITY_MODES) or any(mode not in MODE_NAMES for mode in modes):
        raise ValueError("配置含不支持的模式或缺少 AMP/感知行走")
    bounds = []
    for mode, profile_path in config["policies"].items():
        if int(mode) not in VELOCITY_MODES:
            continue
        with (config_path.parent / profile_path).open() as stream:
            profile = yaml.safe_load(stream)
        low, high = profile["velocity_min"], profile["velocity_max"]
        if len(low) != 3 or len(high) != 3:
            raise ValueError("速度限值必须包含 vx/vy/wz 三项")
        if any(not math.isfinite(float(v)) for v in low + high):
            raise ValueError("速度限值必须为有限值")
        bounds.append((low, high))
    low = [max(pair[0][i] for pair in bounds) for i in range(3)]
    high = [min(pair[1][i] for pair in bounds) for i in range(3)]
    if any(not lo < 0 < hi for lo, hi in zip(low, high)):
        raise ValueError("速度限值交集必须包含零")
    timeout = float(config["timing"]["velocity_timeout"])
    settings["velocity_timeout"] = timeout
    settings.update(modes=modes, low=low, high=high,
                    remote_source=config["remote"]["source"],
                    topic=config["remote"]["velocity_topic"],
                    service=config["remote"]["mode_service"])
    return settings


class MotionFileBox(QtWidgets.QLineEdit):
    selected = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setReadOnly(True)
        self.setAcceptDrops(True)
        self.setPlaceholderText("拖入原始 CSV 或 Motion Stream CSV，或点击浏览")

    def keyPressEvent(self, event):
        if event.matches(QtGui.QKeySequence.Paste):
            mime = QtWidgets.QApplication.clipboard().mimeData()
            urls = mime.urls()
            if len(urls) == 1 and urls[0].isLocalFile():
                self.selected.emit(urls[0].toLocalFile())
            elif mime.hasText():
                self.selected.emit(mime.text().strip().strip("\"'"))
            return
        super().keyPressEvent(event)

    def dragEnterEvent(self, event):
        urls = event.mimeData().urls()
        if len(urls) == 1 and urls[0].isLocalFile():
            event.acceptProposedAction()

    def dropEvent(self, event):
        urls = event.mimeData().urls()
        if len(urls) == 1 and urls[0].isLocalFile():
            self.selected.emit(urls[0].toLocalFile())
            event.acceptProposedAction()


def check_motion_stream_file(path):
    """Validate a 44-slot playback file without changing any offline resource."""
    path = Path(path).expanduser().resolve(strict=True)
    if not path.is_file() or path.suffix.lower() != ".csv":
        raise ValueError("请选择一个 CSV 文件")
    with path.open(encoding="utf-8-sig", newline="") as stream:
        rows = csv.reader(stream)
        header = next(rows, [])
        if len(header) != 270 or header[0] != "timestamp" or any(
                f"joint.pos[{i}]" not in header for i in range(44)):
            raise ValueError("需要 44 槽位 Motion Stream CSV；Motion Editor CSV 请先转换")
        positions = [header.index(f"joint.pos[{i}]") for i in range(29, 41)]
        previous = -1.0
        count = 0
        for count, row in enumerate(rows, 1):
            values = [float(value) for value in row]
            if len(values) != len(header) or not all(map(math.isfinite, values)):
                raise ValueError(f"第 {count + 1} 行列数错误或含非有限值")
            if values[0] < 0 or (count > 1 and values[0] <= previous):
                raise ValueError(f"第 {count + 1} 行时间戳必须递增且非负")
            if any(not 0 <= values[i] <= 1 for i in positions):
                raise ValueError(f"第 {count + 1} 行夹爪控制量不在 0～1 范围")
            previous = values[0]
        if count < 2:
            raise ValueError("动作至少需要两帧")
    return path, count


class Joystick(QtWidgets.QWidget):
    changed = QtCore.pyqtSignal(float, float)

    def __init__(self, low, high, auto_center):
        super().__init__()
        self.low, self.high, self.auto_center = low, high, auto_center
        self.vx = self.wz = 0.0
        self.setMinimumSize(180, 180)
        self.setMaximumHeight(220)

    def set_values(self, vx, wz):
        self.vx, self.wz = vx, wz
        self.update()

    def radius(self):
        return max(1, min(self.width(), self.height()) / 2 - 16)

    def normalized(self, value, axis):
        return value / (self.high[axis] if value >= 0 else -self.low[axis])

    def move_stick(self, position):
        forward = (self.height() / 2 - position.y()) / self.radius()
        turn = (self.width() / 2 - position.x()) / self.radius()
        length = max(1.0, math.hypot(forward, turn))
        forward, turn = forward / length, turn / length
        self.vx = forward * (self.high[0] if forward >= 0 else -self.low[0])
        self.wz = turn * (self.high[1] if turn >= 0 else -self.low[1])
        self.changed.emit(self.vx, self.wz)
        self.update()

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.move_stick(event.pos())

    def mouseMoveEvent(self, event):
        if event.buttons() & QtCore.Qt.LeftButton:
            self.move_stick(event.pos())

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton and self.auto_center:
            self.set_values(0, 0)
            self.changed.emit(0, 0)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        center = QtCore.QPointF(self.width() / 2, self.height() / 2)
        radius = self.radius()
        painter.setPen(QtGui.QPen(QtGui.QColor("#9aa7b0"), 1))
        painter.setBrush(QtGui.QColor("#f2f5f7"))
        painter.drawEllipse(center, radius, radius)
        painter.drawLine(center + QtCore.QPointF(-radius, 0), center + QtCore.QPointF(radius, 0))
        painter.drawLine(center + QtCore.QPointF(0, -radius), center + QtCore.QPointF(0, radius))
        tip = center + QtCore.QPointF(-self.normalized(self.wz, 1) * radius,
                                      -self.normalized(self.vx, 0) * radius)
        painter.setPen(QtGui.QPen(QtGui.QColor("#207baf"), 2))
        painter.drawLine(center, tip)
        painter.setBrush(QtGui.QColor("#207baf"))
        painter.drawEllipse(tip, 9, 9)
        painter.drawText(8, 15, "前进 ↑   左转（逆时针）← / → 右转（顺时针）")


class RemoteWindow(QtWidgets.QWidget):
    def __init__(self, node, settings, parent_pid=None):
        super().__init__()
        self.node, self.settings = node, settings
        self.quadruped = settings["quadruped"]
        self.mode_names = settings["mode_names"]
        self.velocity_modes = settings["velocity_modes"]
        self.parent_pid = parent_pid
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=(ReliabilityPolicy.RELIABLE if self.quadruped
                                      else ReliabilityPolicy.BEST_EFFORT),
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = node.create_publisher(LocoVelocity, settings["topic"], qos)
        self.xbox = Xbox(quadruped=self.quadruped)
        self.xbox_sending = False
        self.xbox_external = False
        self.remote_deadline = 0.0
        self.remote_devices = []
        self.remote_active = False
        self.sbus_status_valid = settings["remote_source"] == "sbus" # Unknown until QUERY.
        self.velocity_subscription = node.create_subscription(
            LocoVelocity, settings["topic"], self.observe_velocity,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # Match the controller's service history; default depth 10 can drop
        # batched replies before the GUI executor drains them.
        service_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=64,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE)
        self.client = node.create_client(LocomotionModeSwitch, settings["service"],
                                         qos_profile=service_qos)
        self.motion_client = (node.create_client(MotionCmd, settings["motion_cmd_service"],
                                                 qos_profile=service_qos)
                              if MotionCmd is not None and settings["offline_actions"] else None)
        self.motion = None
        from file_playback import FilePlayback
        self.file_player = FilePlayback(self)
        self.query = None
        self.mode_requests = []
        self.mode_request_serial = 0
        self.last_status = self.last_query = 0.0
        self.last_query_status = 0.0
        self.eqr_query_ok = False
        self.ready = False
        self.last_tick = time.monotonic()
        self.mode = None
        self.fault = True
        self.sending = False
        self.zero_since = None
        self.closed = False
        self.depth_process = None
        self.depth_attempted = False
        self.depth_stopping_at = None
        model = ("EQR11 四足" if self.quadruped else
                 "AIQ11" + (" " + settings["robot_identity"] if settings.get("robot_identity") else ""))
        self.setWindowTitle(model + " · 基础遥控")
        self.resize(520, 420)
        layout = QtWidgets.QVBoxLayout(self)
        self.status = QtWidgets.QLabel("等待控制器状态；不会自动发送速度")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.owner_status = QtWidgets.QLabel("控制来源：GUI")
        self.owner_status.setWordWrap(True)
        layout.addWidget(self.owner_status)
        xbox_help = QtWidgets.QLabel("手柄：BACK+B 到位 / BACK+A 行走 / LT+RT 阻尼\n"
                                    "BACK+方向键：MIMIC A～D；START+左：MIMIC E\n"
                                    "方向键单独按下再松开：离线动作 11～14")
        xbox_help.setWordWrap(True)
        if self.quadruped:
            xbox_help.setText("手柄：BACK+B 到位 / BACK+A TROT / LT+RT 阻尼\n"
                              "左摇杆平移，右摇杆转向；其余 RL 拔出手柄后在 GUI 选择\n"
                              "RL 需控制器加载相应策略，READY 到位后才能进入")
        layout.addWidget(xbox_help)
        self.command_result = QtWidgets.QLabel("模式请求结果将显示在这里")
        self.command_result.setWordWrap(True)
        layout.addWidget(self.command_result)
        self.detail = QtWidgets.QPlainTextEdit()
        self.detail.setReadOnly(True)
        self.detail.setMaximumHeight(100)
        layout.addWidget(self.detail)
        modes = QtWidgets.QHBoxLayout()
        self.mode_select = QtWidgets.QComboBox()
        for mode in settings["modes"]:
            self.mode_select.addItem(self.mode_names[mode], mode)
        modes.addWidget(self.mode_select)
        self.apply = QtWidgets.QPushButton("切换模式")
        self.apply.clicked.connect(lambda: self.request_mode(self.mode_select.currentData()))
        modes.addWidget(self.apply)
        layout.addLayout(modes)
        actions = QtWidgets.QHBoxLayout()
        self.action_select = QtWidgets.QComboBox()
        for action in settings["offline_actions"]:
            self.action_select.addItem(action["name"], action["cmd"])
        actions.addWidget(self.action_select)
        self.action_trigger = QtWidgets.QPushButton("执行离线动作")
        self.action_trigger.clicked.connect(lambda: self.request_action())
        actions.addWidget(self.action_trigger)
        layout.addLayout(actions)
        file_group = QtWidgets.QGroupBox("本地动作文件（原始 CSV 自动转换）")
        file_layout = QtWidgets.QHBoxLayout(file_group)
        self.motion_file = MotionFileBox()
        self.motion_file.selected.connect(self.select_motion_file)
        file_layout.addWidget(self.motion_file)
        browse = QtWidgets.QPushButton("浏览…")
        browse.clicked.connect(self.browse_motion_file)
        file_layout.addWidget(browse)
        clear = QtWidgets.QPushButton("清除")
        clear.clicked.connect(lambda: self.select_motion_file(""))
        file_layout.addWidget(clear)
        layout.addWidget(file_group)
        file_buttons = QtWidgets.QHBoxLayout()
        self.file_execute = QtWidgets.QPushButton("执行文件")
        self.file_execute.clicked.connect(self.file_player.start)
        self.file_stop = QtWidgets.QPushButton("停止文件")
        self.file_stop.clicked.connect(lambda: self.file_player.stop("已停止文件回放"))
        file_buttons.addWidget(self.file_execute)
        file_buttons.addWidget(self.file_stop)
        layout.addLayout(file_buttons)
        self.file_result = QtWidgets.QLabel("独立文件回放；不占用离线动作编号")
        self.file_result.setWordWrap(True)
        layout.addWidget(self.file_result)
        self.action_status = QtWidgets.QLabel("离线动作服务未连接")
        layout.addWidget(self.action_status)
        self.action_result = QtWidgets.QLabel("选择动作后点击执行；不会自动切换模式")
        self.action_result.setWordWrap(True)
        layout.addWidget(self.action_result)
        self.joystick = Joystick([settings["low"][i] for i in (0, 2)],
                                 [settings["high"][i] for i in (0, 2)], settings["auto_center"])
        layout.addWidget(self.joystick)
        axes = QtWidgets.QFormLayout()
        self.axes, self.sliders = [], []
        for i, label in enumerate(("前进 vx (m/s)", "横移 vy (m/s)", "转向 wz (rad/s)")):
            row = QtWidgets.QHBoxLayout()
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(math.ceil(settings["low"][i] * 1000),
                            math.floor(settings["high"][i] * 1000))
            slider.setValue(0)
            if i == 1:
                slider.setInvertedAppearance(True)
                slider.setInvertedControls(True)
                slider.setToolTip("向左为 +Y，向右为 -Y")
            box = QtWidgets.QDoubleSpinBox()
            box.setDecimals(3)
            box.setRange(settings["low"][i], settings["high"][i])
            box.setSingleStep(settings["velocity_step"])
            slider.valueChanged.connect(lambda value, b=box: b.setValue(value / 1000))
            box.valueChanged.connect(lambda value, s=slider: s.setValue(round(value * 1000)))
            if i == 1:
                row.addWidget(QtWidgets.QLabel("+Y"))
            row.addWidget(slider)
            if i == 1:
                row.addWidget(QtWidgets.QLabel("−Y"))
            row.addWidget(box)
            axes.addRow(label, row)
            self.axes.append(box)
            self.sliders.append(slider)
        self.joystick.changed.connect(self.set_drive)
        for box in (self.axes[0], self.axes[2]):
            box.valueChanged.connect(lambda _: self.joystick.set_values(
                self.axes[0].value(), self.axes[2].value()))
        layout.addLayout(axes)
        self.auto_center = QtWidgets.QCheckBox("自动回零（摇杆 / 滑块松开后归零）")
        self.auto_center.setChecked(settings["auto_center"])
        self.auto_center.toggled.connect(self.set_auto_center)
        layout.addWidget(self.auto_center)
        for i, slider in enumerate(self.sliders):
            slider.sliderReleased.connect(lambda axis=i: self.center_axis_on_release(axis))
        controls = QtWidgets.QHBoxLayout()
        zero = QtWidgets.QPushButton("速度归零")
        zero.clicked.connect(self.zero_velocity)
        controls.addWidget(zero)
        still = QtWidgets.QPushButton("切回 STILL")
        still.clicked.connect(lambda: self.request_mode(1))
        controls.addWidget(still)
        layout.addLayout(controls)
        self.depth_status = QtWidgets.QLabel("深度窗口已关闭")
        layout.addWidget(self.depth_status)
        if self.quadruped:
            for widget in (self.action_select, self.action_trigger, self.action_status,
                           self.action_result, self.depth_status):
                widget.hide()
            self.status_subscription = node.create_subscription(
                String, settings["status_topic"], self.observe_eqr_status, 1)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(max(1, round(1000 / settings["publish_hz"])))
        self.refresh_controls()

    def publish_velocity(self, values):
        if self.remote_active:
            return
        message = LocoVelocity()
        message.priority = LocoVelocity.PRIORITY_NORMAL
        message.linear_vx, message.linear_vy, message.angular_wz = map(float, values)
        message.keep_time = self.settings["keep_time"]
        self.publisher.publish(message)

    def can_send(self):
        return (not self.file_player.active and not self.remote_active and self.motion is None
                and not self.mode_requests and self.mode in self.velocity_modes and not self.fault
                and self.healthy()
                and not self.competing_publishers())

    def healthy(self):
        now = time.monotonic()
        return (self.client.service_is_ready()
                and now - self.last_status < self.settings["status_timeout"]
                and (not self.quadruped or (self.eqr_query_ok and
                     now - self.last_query_status < self.settings["status_timeout"])))

    def observe_eqr_status(self, message):
        # Fault descriptions contain spaces; QUERY's "accepted" is not a status snapshot.
        match = re.fullmatch(r"mode=(\d+) ready=([01]) fault=(.+?) sequence=(\d+)(?: .*)?",
                             message.data)
        if match is None:
            self.last_status, self.fault, self.ready = 0.0, True, False
            self.stop_xbox()
            self.stop_velocity(center=True)
            self.detail.setPlainText("无法解析四足状态：" + message.data)
            return
        mode, ready, fault, _ = match.groups()
        mode, fault = int(mode), fault != "none"
        if self.mode != mode or fault:
            self.stop_xbox()
            self.stop_velocity(center=True)
        if self.mode != mode and mode in EQR_VELOCITY_LIMITS:
            high = list(EQR_VELOCITY_LIMITS[mode])
            low = [-v for v in high]
            self.settings.update(low=low, high=high)
            for i, (box, slider) in enumerate(zip(self.axes, self.sliders)):
                box.setRange(low[i], high[i])
                slider.setRange(round(low[i] * 1000), round(high[i] * 1000))
            self.joystick.low = [low[i] for i in (0, 2)]
            self.joystick.high = [high[i] for i in (0, 2)]
        self.mode, self.ready, self.fault = mode, ready == "1", fault
        self.last_status = time.monotonic()
        self.detail.setPlainText(message.data)
        self.status.setText("故障锁定；排障后重启控制器" if fault else
                            "当前模式：" + self.mode_names.get(mode, str(mode)) +
                            (("（已到位）" if self.ready else "（到位中）") if mode == 2 else ""))

    def observe_velocity(self, message):
        if message.priority == LocoVelocity.PRIORITY_REMOTE:
            valid = all(math.isfinite(v) for v in (message.keep_time, message.linear_vx,
                                                   message.linear_vy, message.angular_wz))
            self.remote_deadline = (time.monotonic() + min(message.keep_time,
                                    self.settings["velocity_timeout"])
                                    if valid and message.keep_time > 0 else 0.0)
            # Observe each transition, including a remote command immediately
            # followed by release in the same callback batch. Never revive GUI values.
            self.sync_remote()

    def competing_publishers(self):
        endpoints = self.node.get_publishers_info_by_topic(self.settings["topic"])
        # A known remote driver may remain alive after USB removal or at rest.
        return sum(endpoint.node_namespace.rstrip("/") + "/" + endpoint.node_name
                   not in self.settings["remote_publisher_nodes"] for endpoint in endpoints) > 1

    def sync_remote(self):
        self.remote_devices = sorted({p for pattern in self.settings["remote_device_globs"]
                                      for p in glob.glob(pattern) if os.path.exists(p)})
        sbus = self.settings["remote_source"] == "sbus" and self.sbus_status_valid
        device_active = (self.xbox.connected if self.settings["xbox_enabled"] and not self.xbox_external
                         else bool(self.remote_devices))
        active = sbus or device_active or time.monotonic() < self.remote_deadline
        if active != self.remote_active:
            self.remote_active = active
            self.stop_velocity(center=True)
            if active and self.motion is not None:
                self.discard(self.motion, self.motion_client)
                self.motion = None
                self.action_result.setText("遥控器接管；已发送的动作请求可能已执行，请核对状态")
        self.owner_status.setText("SBUS 状态未知（GUI 保持让出控制）" if sbus and
                                  time.monotonic() - self.last_status >= self.settings["status_timeout"] else
                                  "控制来源：SBUS（通道有效，GUI 已让出控制）" if sbus else
                                  ("控制来源：Xbox（左摇杆平移 / 右摇杆转向）" if self.xbox.armed else
                                   "Xbox 已连接：请松开所有按键并回中摇杆") if self.xbox.connected else
                                  "控制来源：外部遥控器（GUI 已让出控制）" if active else
                                  self.xbox.error + "；GUI 可用" if self.xbox.error else
                                  "控制来源：GUI（拔出后需重新操作）")

    def stop_xbox(self):
        self.xbox.disarm()
        if self.xbox_sending:
            message = LocoVelocity()
            message.priority = LocoVelocity.PRIORITY_REMOTE
            message.keep_time = self.settings["keep_time"] if self.quadruped else 0.0
            self.publisher.publish(message)
            self.xbox_sending = False

    def update_xbox(self, healthy, competing):
        # Do not run two drivers for the same joystick. Runtime is the SBUS owner.
        external = any(endpoint.node_name != "eir_ctrl_demo_aiq11" and
                       endpoint.node_namespace.rstrip("/") + "/" + endpoint.node_name
                       in self.settings["remote_publisher_nodes"]
                       for endpoint in self.node.get_publishers_info_by_topic(self.settings["topic"]))
        self.xbox_external = external
        if not self.settings["xbox_enabled"] or external:
            self.stop_xbox()
            self.xbox.close()
            return
        actions = self.xbox.poll(self.remote_devices, self.settings["velocity_deadband"])
        sbus = self.settings["remote_source"] == "sbus" and self.sbus_status_valid
        if ("mode", 1) in actions:
            self.stop_xbox()
            self.request_mode(1, from_xbox=True)
            return
        if not healthy or competing or sbus or self.fault or self.mode_requests or self.motion is not None:
            self.stop_xbox()
            return
        if actions and self.xbox.armed:
            kind, cmd = actions[0]
            self.stop_xbox()
            if kind == "motion":
                self.request_action(cmd, from_xbox=True)
            elif cmd not in self.settings["modes"]:
                self.command_result.setText(f"手柄请求 {cmd}：当前控制配置不支持；故障锁定保持有效")
            elif self.xbox.neutral(self.settings["velocity_deadband"]):
                self.request_mode(cmd, from_xbox=True)
            else:
                self.command_result.setText("模式切换前请松开按键、回中摇杆后重新按组合键")
            return
        if not self.xbox.connected or self.mode not in self.velocity_modes:
            if self.xbox_sending:
                self.stop_xbox()
            return
        values = self.xbox.velocity(self.settings["low"], self.settings["high"],
                                    self.settings["velocity_deadband"])
        if any(values) or self.xbox_sending:
            message = LocoVelocity()
            message.priority = LocoVelocity.PRIORITY_REMOTE
            message.linear_vx, message.linear_vy, message.angular_wz = values
            message.keep_time = self.settings["keep_time"] if any(values) or self.quadruped else 0.0
            self.publisher.publish(message)
            self.xbox_sending = any(values)

    def browse_motion_file(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "选择离线动作", self.motion_file.text() or str(Path.home()), "CSV 动作 (*.csv)")
        if path:
            self.select_motion_file(path)

    def select_motion_file(self, path):
        if self.file_player.active:
            self.file_result.setText("请先停止回放，再更换文件")
            return
        if not path:
            self.motion_file.clear()
            self.file_result.setText("未选择文件")
            return
        try:
            from file_playback import load_playback_file
            path, frames, converted = load_playback_file(path)
            count = len(frames)
        except Exception as error:
            self.motion_file.clear()
            self.file_result.setText(f"文件未选中：{error}")
            return
        self.motion_file.setText(str(path))
        self.motion_file.setToolTip(str(path))
        self.file_result.setText(f"已选择 {path.name}，{count} 帧"
                                 + ("（已自动转换为 44 槽位）" if converted else "（Motion Stream）")
                                 + "；点击执行文件开始回放")

    def request_action(self, cmd=None, from_xbox=False):
        if self.file_player.active:
            self.action_result.setText("请先停止文件回放")
            return
        self.sync_remote()
        if self.remote_active and not from_xbox:
            self.action_result.setText("实体遥控器优先，GUI 不发送离线动作")
            return
        if self.competing_publishers():
            self.action_result.setText("存在未识别的速度发布器，不能触发动作")
            return
        if self.motion is not None or self.mode_requests:
            self.action_result.setText("上一次动作或模式请求尚未完成")
            return
        if self.motion_client is None or not self.motion_client.service_is_ready():
            self.action_result.setText("离线动作服务未连接；当前控制器可能不支持 MotionCmd")
            return
        if self.fault or time.monotonic() - self.last_status >= self.settings["status_timeout"]:
            self.action_result.setText("控制器故障或状态过期，不能触发动作")
            return
        if self.settings.get("motion_mode") is not None and self.mode != self.settings["motion_mode"]:
            self.action_result.setText("离线动作仅支持 AMP WALK；请先切换到 AMP WALK")
            return
        cmd = cmd if from_xbox else self.action_select.currentData()
        if cmd is None:
            return
        self.stop_velocity(center=True)
        request = MotionCmd.Request()
        request.cmd, request.session_id = cmd, ""
        self.motion = self.motion_client.call_async(request), time.monotonic()
        self.action_result.setText(f"请求离线动作 cmd={cmd}…")
        self.refresh_controls()

    def set_drive(self, vx, wz):
        self.axes[0].setValue(vx)
        self.axes[2].setValue(wz)

    def set_auto_center(self, enabled):
        self.settings["auto_center"] = enabled
        self.joystick.auto_center = enabled

    def center_axis_on_release(self, axis):
        if self.auto_center.isChecked():
            self.axes[axis].setValue(0)

    def stop_velocity(self, center=False):
        was_sending = self.sending
        self.sending = False
        self.zero_since = None
        if center or self.settings["auto_center"]:
            for box in self.axes:
                box.setValue(0)
        # Explicit zero plus short leases: loss of this message still expires old velocity.
        if was_sending:
            self.publish_velocity((0, 0, 0))

    def zero_velocity(self):
        for box in self.axes:
            box.setValue(0)
        self.update_velocity()

    def update_velocity(self):
        if not self.can_send():
            if self.sending:
                self.stop_velocity(center=True)
            return
        now = time.monotonic()
        values = []
        for i, box in enumerate(self.axes):
            value = box.value()
            limit = self.settings["high"][i] if value >= 0 else -self.settings["low"][i]
            values.append(value if abs(value) > limit * self.settings["velocity_deadband"] else 0.0)
        if any(values):
            self.sending = True
            self.zero_since = None
        elif self.sending:
            if self.zero_since is None:
                self.zero_since = now
            if now - self.zero_since > self.settings["zero_stop_seconds"]:
                self.sending = False
                self.zero_since = None
        if self.sending:
            self.publish_velocity(values)

    def discard(self, pending, client=None):
        if pending is not None:
            (client or self.client).remove_pending_request(pending[0])
            pending[0].cancel()

    def call(self, mode):
        request = LocomotionModeSwitch.Request()
        request.cmd = mode
        started = time.monotonic()
        return self.client.call_async(request), started

    def request_mode(self, mode, from_xbox=False):
        self.file_player.stop("模式切换，停止文件回放")
        if mode not in self.settings["modes"]:
            self.command_result.setText(f"当前机型不支持模式 {mode}")
            return
        if self.quadruped and mode != 1:
            if not self.healthy() or self.fault:
                self.command_result.setText("控制器故障或状态过期；不能切换运动模式")
                return
            if mode in self.velocity_modes and not (self.mode in self.velocity_modes
                                                     or self.mode == 2 and self.ready):
                self.command_result.setText("请先切换 READY 并等待到位，再进入 RL")
                return
        self.sync_remote()
        if self.remote_active and mode != 1 and not from_xbox:
            self.command_result.setText("实体遥控器优先，GUI 不发送运动模式请求")
            return
        if mode != 1 and self.competing_publishers():
            self.command_result.setText("存在未识别的速度发布器，不发送运动模式请求")
            return
        if self.motion is not None and mode != 1:
            self.command_result.setText("离线动作请求尚未完成")
            return
        if mode == 1:
            self.stop_xbox()
        if mode == 1 and self.motion is not None:
            self.discard(self.motion, self.motion_client)
            self.motion = None
            self.action_result.setText("已请求 STILL；先前动作请求可能已执行，请核对状态")
        self.stop_velocity(center=True)
        if self.mode_requests and mode != 1:
            self.command_result.setText("上一次模式请求尚未完成")
            return
        if not self.client.service_is_ready():
            self.command_result.setText("模式服务未连接")
            return
        # Send every STILL immediately, retaining earlier requests until each
        # response or timeout. The controller determines execution priority.
        self.discard(self.query)
        self.query = None
        self.last_status = 0.0
        pending = self.call(mode)
        self.mode_request_serial += 1
        self.mode_requests.append((self.mode_request_serial, mode, pending))
        self.command_result.setText(f"请求 #{self.mode_request_serial} {self.mode_names[mode]}…")
        self.refresh_controls()

    def read_response(self, pending):
        future, started = pending
        if future.done():
            return future.result()
        if time.monotonic() - started >= self.settings["service_timeout"]:
            self.discard(pending)
            raise TimeoutError("服务响应超时；请求可能已应用，请查看实际模式")
        return None

    def read_mode_responses(self):
        for item in self.mode_requests[:]:
            request_id, mode, pending = item
            try:
                response = self.read_response(pending)
                if response is None:
                    continue
                result = ("请求已接受（以实际状态为准）：" if response.result and self.quadruped else
                          "切换已应用：" if response.result else "切换被拒绝：") + response.fb_msg
            except Exception as error:
                result = str(error)
                self.stop_xbox()
                self.stop_velocity(center=True)
                self.last_status = 0.0
            self.mode_requests.remove(item)
            elapsed_ms = (time.monotonic() - pending[1]) * 1000
            message = f"模式请求 #{request_id} cmd={mode} {result}（{elapsed_ms:.1f} ms）"
            self.command_result.setText(message)
            self.node.get_logger().info(message)

    def refresh_controls(self):
        enabled = self.can_send()
        self.joystick.setEnabled(enabled)
        for box, slider in zip(self.axes, self.sliders):
            box.setEnabled(enabled)
            slider.setEnabled(enabled)
        self.apply.setEnabled(not self.remote_active and self.motion is None
                              and not self.mode_requests and self.client.service_is_ready()
                              and not self.competing_publishers())
        self.action_trigger.setEnabled(not self.remote_active and not self.fault
                                       and self.motion is None and not self.mode_requests
                                       and time.monotonic() - self.last_status < self.settings["status_timeout"]
                                       and self.motion_client is not None
                                       and self.motion_client.service_is_ready()
                                       and (self.settings.get("motion_mode") is None
                                            or self.mode == self.settings["motion_mode"])
                                       and not self.competing_publishers()
                                       and self.action_select.count() > 0)
        self.file_execute.setEnabled(bool(self.motion_file.text()) and self.can_send()
                                     and self.motion_client is not None
                                     and self.motion_client.service_is_ready())
        self.file_stop.setEnabled(self.file_player.active)
        if self.file_player.active:
            self.action_trigger.setEnabled(False)
        if self.motion_client is None:
            self.action_status.setText("未安装 MotionCmd 接口或未配置离线动作")
        elif self.motion_client.service_is_ready() and self.settings.get("motion_mode") is not None:
            self.action_status.setText("离线动作服务已连接；仅支持 AMP WALK")
        else:
            self.action_status.setText("离线动作服务已连接" if self.motion_client.service_is_ready()
                                       else "离线动作服务未连接")

    def sync_depth_viewer(self, healthy):
        desired = (healthy and not self.fault and self.mode == 41 and not self.mode_requests
                   and self.settings["open_depth_viewer"])
        process = self.depth_process
        if process is not None and process.poll() is not None:
            if self.depth_stopping_at is None and process.returncode != 0:
                self.node.get_logger().warning("深度窗口启动失败或异常退出，请检查 rqt_image_view")
            self.depth_process = None
            self.depth_stopping_at = None
        if not desired:
            self.depth_attempted = False
            if self.depth_process is not None and self.depth_stopping_at is None:
                self.depth_process.terminate()
                self.depth_stopping_at = time.monotonic()
        elif self.depth_process is None and not self.depth_attempted:
            self.depth_attempted = True
            topic = self.settings["depth_image_topic"]
            if not topic:
                self.depth_status.setText("深度窗口已关闭")
                self.node.get_logger().warning("未配置深度图像话题，不打开 rqt")
                return
            try:
                self.depth_process = subprocess.Popen(
                    ["/usr/bin/python3", str(Path(__file__).resolve().parent / "depth_viewer.py"),
                     "--topic", topic, "--parent-pid", str(os.getpid())],
                    stdin=subprocess.DEVNULL, start_new_session=True)
            except OSError as error:
                self.node.get_logger().warning(f"无法打开深度窗口：{error}")
        if self.depth_process is not None and self.depth_stopping_at is not None:
            if time.monotonic() - self.depth_stopping_at > 3:
                try:
                    os.killpg(self.depth_process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self.depth_status.setText("深度窗口已打开" if self.depth_process is not None else
                                  "深度窗口已关闭")

    def close_depth_viewer(self):
        self.sync_depth_viewer(False)
        if self.depth_process is not None:
            try:
                self.depth_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.depth_process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.depth_process.wait()
            self.depth_process = None

    def tick(self):
        if self.closed:
            return
        if self.parent_pid is not None and os.getppid() != self.parent_pid:
            self.close()
            return
        try:
            now = time.monotonic()
            if now - self.last_tick > self.settings["keep_time"]:
                self.stop_xbox()
                self.stop_velocity(center=True)
            self.last_tick = now
            # Drain a bounded batch: our own GUI packets must not hide remote
            # packets when both publishers are active during handover.
            for _ in range(8):
                rclpy.spin_once(self.node, timeout_sec=0)
            self.sync_remote()
            if self.motion is not None:
                future, started = self.motion
                if future.done():
                    response = future.result()
                    self.motion = None
                    self.action_result.setText(("动作请求已接受" if response.result else "动作请求被拒绝")
                                              + f"，错误码 {response.err_code}（不代表动作已完成）")
                elif now - started >= self.settings["service_timeout"]:
                    self.discard(self.motion, self.motion_client)
                    self.motion = None
                    self.action_result.setText("动作服务响应超时；可能已执行，不自动重试")
            self.read_mode_responses()
            if self.query is not None:
                response = self.read_response(self.query)
                if response is not None:
                    self.query = None
                    if not response.result:
                        raise RuntimeError("状态查询被拒绝：" + response.fb_msg)
                    self.last_query_status = now
                    if self.quadruped:
                        # A successful QUERY can still report a latched EQR fault.
                        self.eqr_query_ok = response.fb_msg == "accepted"
                        if response.fb_msg != "accepted":
                            self.fault, self.ready, self.last_status = True, False, 0.0
                            self.stop_xbox()
                            self.stop_velocity(center=True)
                            self.detail.setPlainText(response.fb_msg)
                        elif response.mode != self.mode:
                            self.last_status = 0.0  # Wait for a matching status topic sample.
                    else:
                        self.observe_aiq_response(response, now)
            if self.query is None and not self.mode_requests and now - self.last_query >= self.settings["query_period"]:
                if self.client.service_is_ready():
                    self.query = self.call(0)
                    self.last_query = now
            healthy = self.healthy()
            competing = self.competing_publishers()
            if not healthy or competing:
                if self.sending:
                    self.stop_velocity(center=True)
                if not healthy and self.motion is not None:
                    self.discard(self.motion, self.motion_client)
                    self.motion = None
                    self.action_result.setText("控制器状态过期；动作可能已执行，不自动重试")
                if competing:
                    self.status.setText("检测到其他速度发布器；关闭其他遥控输入后再操作")
                elif self.quadruped and self.fault:
                    self.status.setText("故障或状态无效；速度已归零，请核对控制器状态")
                elif not self.mode_requests:
                    self.status.setText("状态未连接或过期；速度已归零")
            self.file_player.tick(healthy and not competing and not self.remote_active and not self.fault)
            self.update_xbox(healthy, competing)
            self.sync_remote()
            self.update_velocity()
            self.sync_depth_viewer(healthy)
            self.refresh_controls()
        except Exception as error:
            self.file_player.stop(str(error))
            self.stop_xbox()
            self.discard(self.motion, self.motion_client)
            self.motion = None
            self.discard(self.query)
            self.query = None
            self.eqr_query_ok = False
            self.last_status = 0.0
            self.stop_velocity(center=True)
            self.sync_depth_viewer(False)
            self.command_result.setText(str(error))
            self.refresh_controls()

    def observe_aiq_response(self, response, now):
        fault = response.fb_msg.startswith("FAULT")
        if self.mode != response.mode or fault:
            self.stop_xbox()
            self.stop_velocity(center=True)
        self.mode, self.fault, self.last_status = response.mode, fault, now
        if fault and self.motion is not None:
            self.discard(self.motion, self.motion_client)
            self.motion = None
            self.action_result.setText("控制器故障；动作可能已执行，不自动重试")
        # Missing status from an older SBUS controller is ambiguous:
        # keep GUI blocked rather than infer unplug from idle traffic.
        self.sbus_status_valid = (self.settings["remote_source"] == "sbus"
                                  and "sbus_input=unavailable" not in response.fb_msg)
        self.sync_remote()
        self.detail.setPlainText(response.fb_msg)
        self.status.setText("故障锁定；排障后重启控制器" if fault else
                            "当前模式：" + self.mode_names.get(self.mode, str(self.mode)))

    def closeEvent(self, event):
        self.file_player.stop("窗口已关闭")
        self.timer.stop()
        self.stop_xbox()
        self.xbox.close()
        self.stop_velocity(center=True)
        self.close_depth_viewer()
        self.discard(self.query)
        for _, _, pending in self.mode_requests:
            self.discard(pending)
        self.mode_requests.clear()
        self.discard(self.motion, self.motion_client)
        self.closed = True
        event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="控制器使用的同一 YAML")
    parser.add_argument("--gui-config", default=str(Path(__file__).resolve().parent /
                                                   "config/remote_gui.yaml"))
    parser.add_argument("--namespace", help="EQR 控制器命名空间，默认 /eqr11")
    parser.add_argument("--parent-pid", type=int, help=argparse.SUPPRESS)
    args = parser.parse_args()
    settings = load_settings(args.config, args.gui_config, args.namespace)
    app = QtWidgets.QApplication([sys.argv[0]])
    rclpy.init(args=[])
    node = Node("eir_remote_gui")
    window = RemoteWindow(node, settings, args.parent_pid)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    window.show()
    try:
        return app.exec_()
    finally:
        if not window.closed:
            window.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
