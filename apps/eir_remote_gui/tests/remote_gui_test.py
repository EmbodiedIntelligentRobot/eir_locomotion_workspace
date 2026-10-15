#!/usr/bin/python3
# SPDX-License-Identifier: BSD-3-Clause
"""Headless Qt + real local ROS tests; no controller or robot is connected."""
import copy
import importlib.util
import os
import sys
import struct
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.task import Future
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtTest import QTest
import ehr_message.msg as ehr_messages
from ehr_message.msg import LocoVelocity
from ehr_message.srv import LocomotionModeSwitch, MotionCmd
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
spec = importlib.util.spec_from_file_location("remote_gui", ROOT / "remote_gui.py")
gui = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gui)


HAS_CSV_TOOLS = importlib.util.find_spec("motion_stream_tools") is not None
HAS_MOTION_FEEDBACK = all(hasattr(ehr_messages, name) for name in ("MotionState", "MotionExeStatus"))


class RemoteGuiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication([])
        cls.app.setQuitOnLastWindowClosed(False)
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.settings = gui.load_settings(ROOT / "tests/data/controller.yaml",
                                          ROOT / "config/remote_gui.yaml")
        self.settings.update(topic="/gui_test/velocity", service="/gui_test/mode")
        self.settings.update(xbox_enabled=False, remote_device_globs=[], motion_cmd_service="/gui_test/motion")
        self.settings["remote_publisher_nodes"].append("/physical_remote")
        self.settings["open_depth_viewer"] = False
        self.server = Node("gui_test_controller")
        self.node = Node("gui_test_remote")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.server)
        self.mode, self.fault, self.reject, self.drop = 28, False, False, False
        self.sbus_input = None
        self.messages, self.commands = [], []
        self.service = self.server.create_service(LocomotionModeSwitch,
                                                  self.settings["service"], self.handle,
                                                  qos_profile=QoSProfile(depth=64))
        self.motion_commands = []
        self.motion_reject = False
        self.motion_service = self.server.create_service(
            MotionCmd, self.settings["motion_cmd_service"], self.handle_motion)
        self.subscription = self.server.create_subscription(
            LocoVelocity, self.settings["topic"], self.messages.append, qos_profile_sensor_data)
        self.window = gui.RemoteWindow(self.node, self.settings)
        self.window.timer.stop()  # Drive both ROS executors deterministically.
        self.extra = None
        self.pump_until(lambda: self.window.can_send())

    def tearDown(self):
        if not self.window.closed:
            self.window.close()
        if self.extra:
            self.extra.destroy_node()
        self.node.destroy_node()
        self.executor.remove_node(self.server)
        self.executor.shutdown()
        self.server.destroy_node()
        self.app.processEvents()

    def handle(self, request, response):
        self.commands.append(request.cmd)
        response.result = not (self.reject and request.cmd != 0)
        if request.cmd != 0 and response.result:
            self.mode = request.cmd
        response.mode = 1 if self.fault else self.mode
        response.fb_msg = ("FAULT (latched): test" if self.fault else
                           "test ready=false" if response.result else "posture not ready")
        if self.sbus_input is not None:
            response.fb_msg += " sbus_input=" + self.sbus_input
        return response

    def handle_motion(self, request, response):
        self.motion_commands.append((request.cmd, request.session_id))
        response.result = not self.motion_reject
        response.err_code = MotionCmd.Response.ERR_REQUEST_REJECTED if self.motion_reject else 0
        return response

    def pump(self, duration=0.1):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.window.tick()
            if not self.drop:
                self.executor.spin_once(timeout_sec=0)
            self.app.processEvents()
            time.sleep(0.005)

    def pump_until(self, condition, timeout=3):
        end = time.monotonic() + timeout
        while not condition() and time.monotonic() < end:
            self.pump(0.02)
        self.assertTrue(condition())

    def begin(self):
        self.messages.clear()
        self.window.axes[0].setValue(0.4)
        self.window.axes[1].setValue(-0.2)
        self.window.axes[2].setValue(0.3)
        self.pump_until(lambda: any(msg.linear_vx > 0.3 for msg in self.messages))
        self.assertTrue(self.window.sending)

    @unittest.skipUnless(HAS_CSV_TOOLS and HAS_MOTION_FEEDBACK, "requires optional CSV tools and full motion feedback messages")
    def test_direct_file_playback_does_not_change_numbered_actions(self):
        from ehr_message.msg import MotionState, MotionExeStatus
        from motion_stream_tools.csv_codec import MotionFrame, write_motion_csv
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'local.csv'
            write_motion_csv(source, [MotionFrame(i * 0.02, [i * 0.01] * 44)
                                      for i in range(11)], 44)
            original = source.read_bytes()
            actions = copy.deepcopy(self.settings['offline_actions'])
            self.settings['motion_stream_topic'] = '/gui_test/motion_stream'
            received = []
            subscription = self.server.create_subscription(MotionState,
                '/gui_test/motion_stream', received.append, 10)
            status_pub = self.server.create_publisher(MotionExeStatus,
                '/gui_test/motion_exe_status', qos_profile_sensor_data)
            def feedback():
                if self.motion_commands and self.motion_commands[-1][0] == 1:
                    msg = MotionExeStatus()
                    msg.header.frame_id = 'JOINT_REALTIME:REALTIME_TOPIC'
                    msg.state = 1
                    status_pub.publish(msg)
            timer = self.server.create_timer(0.02, feedback)
            mime = QtCore.QMimeData()
            mime.setUrls([QtCore.QUrl.fromLocalFile(str(source))])
            drop = gui.QtGui.QDropEvent(QtCore.QPointF(0, 0), QtCore.Qt.CopyAction,
                                       mime, QtCore.Qt.LeftButton, QtCore.Qt.NoModifier)
            self.window.motion_file.dropEvent(drop)
            self.assertTrue(drop.isAccepted())
            self.window.select_motion_file('')
            gui.QtWidgets.QApplication.clipboard().setText(str(source))
            QTest.keyClick(self.window.motion_file, QtCore.Qt.Key_V, QtCore.Qt.ControlModifier)
            self.assertEqual(self.window.motion_file.text(), str(source))
            self.window.file_player.start()
            self.assertTrue(self.window.file_player.active)
            self.pump_until(lambda: not self.window.file_player.active)
            self.assertEqual([cmd for cmd, session in self.motion_commands], [1, 2])
            self.assertGreater(len(received), 3)
            self.assertEqual(received[-1].frame_type, 1)
            self.assertAlmostEqual(received[-1].joint_states.position[0], 0.1)
            self.assertEqual(source.read_bytes(), original)
            self.assertEqual(self.settings['offline_actions'], actions)
            self.window.request_action()
            self.pump_until(lambda: self.motion_commands[-1][0] > 10)
            source.write_text('bad,csv\n1,2\n')
            self.window.select_motion_file(str(source))
            self.assertFalse(self.window.motion_file.text())
            self.assertIn('未选中', self.window.file_result.text())
            self.server.destroy_timer(timer)
            self.server.destroy_subscription(subscription)
            self.server.destroy_publisher(status_pub)

    @unittest.skipUnless(HAS_CSV_TOOLS, "requires optional motion_stream_tools")
    def test_raw_csv_uses_existing_converter(self):
        import csv
        import numpy as np
        from file_playback import load_playback_file
        from motion_stream_tools.layout import load_any_layout
        from motion_stream_tools.importers import convert_path
        from motion_stream_tools.csv_codec import read_motion_csv
        from motion_stream_tools.aiq11pro_hand_mapping import AIQ11PRO_HAND_MOTORS
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'editor.csv'
            layout = load_any_layout('aiq11pro')
            names = layout.joint_names[:29] + layout.joint_names[41:44]
            motors = [entry[1] for entry in AIQ11PRO_HAND_MOTORS]
            with source.open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow([50])
                writer.writerow(['timestamp'] + names + motors)
                for i in range(3):
                    writer.writerow([i * 0.02] + [0.05 * i] * len(names)
                                    + [entry[2][0][2] for entry in AIQ11PRO_HAND_MOTORS])
            before = source.read_bytes()
            self.window.select_motion_file(str(source))
            self.assertIn('自动转换', self.window.file_result.text())
            _, actual, converted = load_playback_file(source)
            self.assertTrue(converted)
            reference = root / 'reference.csv'
            convert_path(source, reference, layout)
            expected = read_motion_csv(reference, 44)
            for a, b in zip(actual, expected):
                np.testing.assert_array_equal(a.joint_position, b.joint_position)
                self.assertEqual(a.timestamp, b.timestamp)
                self.assertEqual(a.joint_pos_mask, b.joint_pos_mask)
            self.assertEqual(source.read_bytes(), before)
            self.assertEqual(sorted(p.name for p in root.iterdir()), ['editor.csv', 'reference.csv'])
            _, ready, converted = load_playback_file(reference)
            self.assertFalse(converted)
            self.assertEqual(len(ready), 3)
            source.write_text(source.read_text().replace('0.05', 'nan', 1))
            with self.assertRaises(ValueError):
                load_playback_file(source)

    def assert_zero(self, stopped=True):
        self.pump_until(lambda: bool(self.messages) and self.messages[-1].linear_vx == 0)
        message = self.messages[-1]
        self.assertEqual((message.linear_vy, message.angular_wz), (0, 0))
        self.assertEqual(self.window.sending, not stopped)

    def fake_xbox(self, held=False):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        device = Path(folder.name) / "js0"
        device.touch()
        reader, writer = os.pipe2(os.O_NONBLOCK)
        self.addCleanup(os.close, writer)
        xbox = self.window.xbox
        xbox.fd, xbox.path = reader, str(device)
        xbox.axis_map = [0, 1, 3, 2, 4, 5, 16, 17]
        xbox.button_map = [304, 305, 307, 310, 308, 311, 314, 315]
        self.settings.update(xbox_enabled=True, remote_device_globs=[str(device)])
        def event(kind, index, value):
            os.write(writer, struct.pack('IhBB', 0, value, kind, index))
        for axis in range(8):
            event(0x82, axis, -32768 if axis in (3, 5) else
                  -16000 if held and axis == 1 else 0)
        for button in range(8):
            event(0x81, button, int(held and button == 3))
        self.pump(0.05)
        return xbox, device, event

    def test_xbox_mode_combos_direct_velocity_and_unplug(self):
        xbox, device, event = self.fake_xbox()
        self.assertTrue(xbox.armed)
        self.assertTrue(self.window.remote_active)
        event(1, 6, 1)
        event(1, 1, 1)  # BACK+B -> READY
        self.pump_until(lambda: 2 in self.commands and self.window.mode == 2)
        event(1, 1, 0)
        event(1, 6, 0)
        self.pump_until(lambda: xbox.armed)
        event(1, 6, 1)
        event(1, 0, 1)  # BACK+A -> AMP
        self.pump_until(lambda: 28 in self.commands and self.window.mode == 28)
        event(1, 0, 0)
        event(1, 6, 0)
        self.pump_until(lambda: xbox.armed)
        self.messages.clear()
        event(2, 1, -16000)
        self.pump_until(lambda: any(m.linear_vx > 0 for m in self.messages))
        self.assertTrue(all(m.priority == 0 for m in self.messages))
        event(2, 1, 0)
        self.pump_until(lambda: self.messages[-1].linear_vx == 0)
        self.assertEqual(self.messages[-1].keep_time, 0)
        event(2, 1, -16000)
        self.pump_until(lambda: self.messages[-1].linear_vx > 0)
        device.unlink()
        self.pump_until(lambda: self.window.can_send())
        self.pump_until(lambda: self.messages[-1].linear_vx == 0)
        self.assertFalse(self.window.sending)

    def test_xbox_held_on_connect_requires_release_and_neutral(self):
        xbox, _, event = self.fake_xbox(held=True)
        self.assertFalse(xbox.armed)
        self.assertFalse(any(m.linear_vx for m in self.messages))
        self.assertEqual([c for c in self.commands if c], [])
        event(1, 3, 0)
        self.pump()
        self.assertFalse(xbox.armed)
        event(2, 1, 0)
        self.pump_until(lambda: xbox.armed)
        event(1, 3, 1)
        event(2, 1, -32768)
        self.pump_until(lambda: any(m.linear_vx > 0 for m in self.messages))
        self.assertAlmostEqual(self.messages[-1].linear_vx, self.settings["high"][0])

    def test_xbox_status_loss_and_stall_do_not_resume_held_velocity(self):
        xbox, _, event = self.fake_xbox()
        event(1, 3, 1)
        event(2, 1, -16000)
        self.pump_until(lambda: self.window.xbox_sending)
        self.window.last_tick -= 1
        self.pump()
        self.assertFalse(xbox.armed)
        self.assertFalse(self.window.xbox_sending)
        event(1, 3, 0)
        event(2, 1, 0)
        self.pump_until(lambda: xbox.armed)
        event(1, 3, 1)
        event(2, 1, -16000)
        self.pump_until(lambda: self.window.xbox_sending)
        self.drop = True
        self.pump(self.settings["status_timeout"] + 0.2)
        self.assertFalse(self.window.xbox_sending)
        self.drop = False
        self.pump(0.4)
        self.assertFalse(xbox.armed)
        self.assertFalse(self.window.xbox_sending)

    def test_xbox_sbus_and_fault_block_mode_and_velocity_but_allow_stop(self):
        xbox, _, event = self.fake_xbox()
        self.settings["remote_source"] = "sbus"
        self.sbus_input = "valid"
        self.pump_until(lambda: self.window.sbus_status_valid)
        event(1, 2, 1)
        event(1, 3, 1)
        event(2, 1, -16000)
        self.pump()
        self.assertFalse(self.window.xbox_sending)
        self.assertNotIn(28, self.commands)
        event(2, 3, 32767)
        event(2, 5, 32767)  # LT+RT remains available for explicit STILL.
        self.pump_until(lambda: 1 in self.commands)
        self.fault = True
        self.sbus_input = "unavailable"
        event(1, 1, 0)
        self.pump_until(lambda: self.window.fault)
        event(1, 0, 1)
        self.pump()
        self.assertNotIn(2, self.commands)
        self.assertFalse(self.window.xbox_sending)

    def test_xbox_axes_signs_and_read_failure_stop(self):
        xbox, _, event = self.fake_xbox()
        event(1, 3, 1)
        event(2, 0, -32768)  # left -> positive lateral
        event(2, 1, 32767)   # down -> backward
        event(2, 2, 32767)   # right -> negative yaw
        self.pump_until(lambda: self.window.xbox_sending)
        self.pump_until(lambda: bool(self.messages))
        message = self.messages[-1]
        self.assertAlmostEqual(message.linear_vx, self.settings["low"][0])
        self.assertAlmostEqual(message.linear_vy, self.settings["high"][1])
        self.assertAlmostEqual(message.angular_wz, self.settings["low"][2])
        with patch("xbox.os.read", side_effect=OSError("disconnected")):
            self.pump_until(lambda: not self.window.xbox_sending)
        self.pump_until(lambda: self.messages[-1].keep_time == 0)
        self.assertFalse(xbox.connected)

    def test_xbox_initial_button_does_not_switch_and_backlog_cannot_move(self):
        xbox, _, event = self.fake_xbox()
        event(0x81, 6, 1)
        event(0x81, 1, 1)
        self.pump()
        self.assertNotIn(2, self.commands)
        event(1, 6, 0)
        event(1, 1, 0)
        event(1, 3, 1)
        for _ in range(260):
            event(2, 1, -16000)
        self.pump()
        self.assertFalse(xbox.armed)
        self.assertFalse(self.window.xbox_sending)
        self.assertFalse(any(m.linear_vx for m in self.messages))

    def xbox_keys(self, event, keys):
        for index, name in enumerate(("A", "B", "X", "LB", "Y", "RB", "BACK", "START")):
            event(1, index, int(name in keys))
        event(2, 3, 32767 if "LT" in keys else -32768)
        event(2, 5, 32767 if "RT" in keys else -32768)
        event(2, 6, -32767 if "LEFT" in keys else 32767 if "RIGHT" in keys else 0)
        event(2, 7, -32767 if "UP" in keys else 32767 if "DOWN" in keys else 0)

    def test_xbox_humanoid_mimic_and_unsupported_shortcuts(self):
        xbox, _, event = self.fake_xbox()
        for keys, mode in gui.Xbox.SHORTCUTS[1:]:
            with self.subTest(keys=keys):
                self.xbox_keys(event, set())
                self.pump_until(lambda: xbox.armed)
                before = len(self.commands)
                self.xbox_keys(event, keys)
                if mode in self.settings["modes"]:
                    self.pump_until(lambda: mode in self.commands[before:])
                    self.pump_until(lambda: not self.window.mode_requests and self.window.mode == mode)
                    self.pump(0.05)
                    self.assertEqual(self.commands[before:].count(mode), 1)
                else:
                    self.pump()
                    self.assertIn("不支持", self.window.command_result.text())
                    self.assertNotIn(mode, self.commands[before:])
        self.assertEqual(self.motion_commands, [])

    def test_xbox_dpad_releases_call_motion_cmd_once(self):
        xbox, _, event = self.fake_xbox()
        for key, cmd in gui.Xbox.SINGLE_PRESSES.items():
            self.pump_until(lambda: xbox.armed)
            before = len(self.motion_commands)
            self.xbox_keys(event, {key})
            self.pump(0.04)
            self.assertEqual(len(self.motion_commands), before)
            self.xbox_keys(event, set())
            self.pump_until(lambda: len(self.motion_commands) == before + 1)
            self.assertEqual(self.motion_commands[-1], (cmd, ""))
            self.pump_until(lambda: self.window.motion is None)
        self.assertEqual([c for c in self.commands if c], [])

    def test_xbox_dpad_click_cancels_on_other_key_or_stick(self):
        xbox, _, event = self.fake_xbox()
        for interference in ("LB", "BACK", "START", "stick", "right_y"):
            self.xbox_keys(event, {"UP"})
            self.pump(0.03)
            if interference in ("stick", "right_y"):
                axis = 1 if interference == "stick" else 4
                event(2, axis, 16000)
                event(2, axis, 0)  # Even a transient within one batch cancels.
            else:
                self.xbox_keys(event, {"UP", interference})
                self.xbox_keys(event, {"UP"})
            self.pump(0.04)
            self.xbox_keys(event, set())
            self.pump_until(lambda: xbox.armed and not self.window.mode_requests)
            self.pump(0.05)
        self.assertEqual(self.motion_commands, [])

    def test_xbox_still_combo_priority_and_single_b_is_not_stop(self):
        xbox, _, event = self.fake_xbox()
        self.xbox_keys(event, {"B"})
        self.pump()
        self.assertNotIn(1, self.commands)
        self.xbox_keys(event, set())
        self.pump()
        before = len(self.commands)
        self.xbox_keys(event, {"BACK", "B", "LT", "RT", "LB", "RB"})
        self.pump_until(lambda: 1 in self.commands[before:])
        self.pump(0.15)
        self.assertEqual([cmd for cmd in self.commands[before:] if cmd], [1])
        self.assertEqual(self.motion_commands, [])

    def test_xbox_external_driver_disables_builtin_reader(self):
        xbox, _, event = self.fake_xbox()
        self.extra = Node("physical_remote")
        publisher = self.extra.create_publisher(LocoVelocity, self.settings["topic"],
                                                qos_profile_sensor_data)
        self.pump_until(lambda: self.window.xbox_external)
        self.assertFalse(xbox.connected)
        self.assertTrue(self.window.remote_active)  # Still plugged in, external owner.
        self.assertFalse(self.window.can_send())
        self.assertFalse(self.window.xbox_sending)
        self.extra.destroy_publisher(publisher)

    def test_xbox_unreadable_device_does_not_claim_control(self):
        with tempfile.TemporaryDirectory() as folder:
            device = Path(folder) / "js0"
            device.touch()
            self.settings.update(xbox_enabled=True, remote_device_globs=[str(device)])
            self.pump()
            self.assertFalse(self.window.xbox.connected)
            self.assertTrue(self.window.can_send())
            self.assertIn("无法读取", self.window.owner_status.text())

    def test_velocity_lease_stop_and_bounds(self):
        self.assertFalse(self.window.sending)
        self.assertFalse(any(msg.linear_vx for msg in self.messages))
        self.begin()
        moving = [msg for msg in self.messages if msg.linear_vx > 0.3][-1]
        self.assertEqual(moving.priority, LocoVelocity.PRIORITY_NORMAL)
        self.assertAlmostEqual(moving.keep_time, self.settings["keep_time"])
        self.window.zero_velocity()
        self.assert_zero(stopped=False)
        self.window.zero_since = time.monotonic() - self.settings["zero_stop_seconds"] - 0.1
        self.pump(0.1)
        self.assertFalse(self.window.sending)
        self.assertEqual([box.value() for box in self.window.axes], [0, 0, 0])
        for box, lo, hi in zip(self.window.axes, self.settings["low"], self.settings["high"]):
            box.setValue(100)
            self.assertEqual(box.value(), hi)
            box.setValue(-100)
            self.assertEqual(box.value(), lo)

    def test_rejected_switch_is_visible_and_stops_velocity(self):
        self.begin()
        self.reject = True
        self.window.request_mode(2)
        self.assert_zero()
        self.pump_until(lambda: not self.window.mode_requests)
        self.pump(0.4)
        self.assertIn("切换被拒绝", self.window.command_result.text())
        self.assertIn("posture not ready", self.window.command_result.text())
        self.assertEqual(self.window.mode, 28)

    def test_modes_and_fault_do_not_resume_velocity(self):
        self.assertEqual([self.window.mode_select.itemData(i) for i in
                          range(self.window.mode_select.count())], self.settings["modes"])
        self.begin()
        self.mode = 30
        self.pump_until(lambda: self.window.mode == 30)
        self.assert_zero()
        self.assertFalse(self.window.joystick.isEnabled())
        self.mode = 41
        self.pump_until(lambda: self.window.mode == 41)
        self.assertFalse(self.window.sending)
        self.begin()
        self.fault = True
        self.pump_until(lambda: self.window.fault)
        self.assert_zero()
        self.assertFalse(self.window.joystick.isEnabled())
        self.assertIn("故障锁定", self.window.status.text())

    def test_status_loss_timeout_and_reconnect(self):
        self.begin()
        self.drop = True
        self.pump(self.settings["status_timeout"] + 0.1)
        self.assertFalse(self.window.sending)
        self.assertFalse(self.window.joystick.isEnabled())
        self.pump(0.4)
        self.assertIn("超时", self.window.command_result.text())
        self.drop = False
        self.pump_until(lambda: self.window.can_send())
        self.assert_zero()

    def test_gui_stall_focus_loss_and_close(self):
        self.begin()
        time.sleep(self.settings["keep_time"] + 0.02)
        self.window.tick()
        self.assert_zero()
        self.begin()
        QtWidgets.QApplication.sendEvent(self.window, QtCore.QEvent(QtCore.QEvent.WindowDeactivate))
        self.pump(0.1)
        self.assertTrue(self.window.sending)
        self.assertGreater(self.messages[-1].linear_vx, 0.3)
        self.window.close()
        self.assert_zero()
        self.assertTrue(self.window.closed)
        self.assertFalse(self.window.timer.isActive())

    def test_joystick_holds_and_optional_auto_center(self):
        stick = self.window.joystick
        position = QtCore.QPoint(stick.width() // 2, stick.height() // 2 - 30)
        QTest.mousePress(stick, QtCore.Qt.LeftButton, pos=position)
        QTest.mouseRelease(stick, QtCore.Qt.LeftButton, pos=position)
        self.assertGreater(self.window.axes[0].value(), 0)
        self.pump_until(lambda: self.window.sending)
        self.assertFalse(self.window.auto_center.isChecked())
        self.window.auto_center.setChecked(True)
        QTest.mousePress(stick, QtCore.Qt.LeftButton, pos=position)
        QTest.mouseRelease(stick, QtCore.Qt.LeftButton, pos=position)
        self.assertEqual([self.window.axes[i].value() for i in (0, 2)], [0, 0])
        self.window.sliders[2].setValue(300)
        self.window.sliders[2].sliderReleased.emit()
        self.assertEqual(self.window.axes[2].value(), 0)
        self.window.auto_center.setChecked(False)
        self.window.sliders[2].setValue(300)
        self.window.sliders[2].sliderReleased.emit()
        self.assertEqual(self.window.axes[2].value(), 0.3)

    def test_joystick_turn_direction_keeps_lateral_slider_independent(self):
        stick = self.window.joystick
        self.window.axes[1].setValue(0.2)
        center = QtCore.QPoint(stick.width() // 2, stick.height() // 2)
        for direction, sign in ((-1, 1), (1, -1)):
            position = center + QtCore.QPoint(direction * 30, 0)
            QTest.mousePress(stick, QtCore.Qt.LeftButton, pos=position)
            QTest.mouseRelease(stick, QtCore.Qt.LeftButton, pos=position)
            self.pump(0.05)
            self.assertGreater(sign * self.messages[-1].angular_wz, 0)
            self.assertAlmostEqual(self.messages[-1].linear_vy, 0.2)
        self.window.auto_center.setChecked(True)
        QTest.mousePress(stick, QtCore.Qt.LeftButton, pos=center + QtCore.QPoint(-30, -30))
        QTest.mouseRelease(stick, QtCore.Qt.LeftButton, pos=center)
        self.assertEqual([self.window.axes[i].value() for i in (0, 2)], [0, 0])
        self.assertEqual(self.window.axes[1].value(), 0.2)

    def test_lateral_slider_left_is_positive_y(self):
        self.window.show()
        self.app.processEvents()
        slider = self.window.sliders[1]
        self.assertTrue(slider.invertedAppearance())
        self.assertTrue(slider.invertedControls())
        slider.setFocus()
        QTest.keyClick(slider, QtCore.Qt.Key_Left)
        self.assertGreater(self.window.axes[1].value(), 0)
        # Real mouse dragging uses the displayed direction, not a changed message sign.
        center = QtCore.QPoint(slider.width() // 2, slider.height() // 2)
        slider.setValue(0)
        QTest.mousePress(slider, QtCore.Qt.LeftButton, pos=center)
        # QTest.mouseMove does not retain the pressed-button mask; provide it explicitly.
        from PyQt5 import QtGui
        for target, sign in ((QtCore.QPoint(5, center.y()), 1),
                             (QtCore.QPoint(slider.width() - 5, center.y()), -1)):
            event = QtGui.QMouseEvent(QtCore.QEvent.MouseMove, target, QtCore.Qt.NoButton,
                                     QtCore.Qt.LeftButton, QtCore.Qt.NoModifier)
            self.app.sendEvent(slider, event)
            self.pump(0.06)
            self.assertGreater(sign * self.messages[-1].linear_vy, 0)
        QTest.mouseRelease(slider, QtCore.Qt.LeftButton, pos=center)

    def test_competing_publisher_blocks_sending(self):
        self.begin()
        self.extra = Node("other_remote")
        self.extra.create_publisher(LocoVelocity, self.settings["topic"], qos_profile_sensor_data)
        self.pump_until(lambda: not self.window.sending)
        self.assert_zero()
        self.assertFalse(self.window.joystick.isEnabled())
        self.assertIn("其他速度发布器", self.window.status.text())
        self.window.request_action()
        self.assertIsNone(self.window.motion)
        self.assertIn("未识别", self.window.action_result.text())
        self.extra.destroy_node()
        self.extra = None
        self.pump_until(lambda: self.window.can_send())
        self.assertFalse(self.window.sending)

    def test_usb_insert_remove_clears_gui_and_blocks_actions(self):
        self.begin()
        with tempfile.TemporaryDirectory() as folder:
            device = Path(folder) / "js0"
            self.settings["remote_device_globs"] = [str(Path(folder) / "js*")]
            device.touch()
            self.pump_until(lambda: self.window.remote_active)
            self.assertFalse(self.window.sending)
            self.assertTrue(all(box.value() == 0 for box in self.window.axes))
            self.assertFalse(self.window.joystick.isEnabled())
            before = len(self.commands)
            self.window.request_mode(30)
            self.window.request_action()
            self.pump()
            self.assertFalse(any(cmd == 30 for cmd in self.commands[before:]))
            self.assertEqual(self.motion_commands, [])
            device.unlink()
            self.pump_until(lambda: self.window.can_send())
            self.assertFalse(self.window.sending)
            self.window.axes[0].setValue(0.2)
            self.pump_until(lambda: self.window.sending)

    def test_runtime_sbus_status_blocks_gui_and_unavailable_allows_new_input(self):
        self.begin()
        self.settings["remote_source"] = "sbus"
        self.sbus_input = "valid"
        self.pump_until(lambda: self.window.remote_active)
        self.assertIn("SBUS", self.window.owner_status.text())
        self.assertFalse(self.window.sending)
        self.sbus_input = "unavailable"
        self.pump_until(lambda: self.window.can_send())
        self.assertFalse(self.window.sending)
        self.assertTrue(all(box.value() == 0 for box in self.window.axes))
        self.sbus_input = None # Older controller cannot prove SBUS is absent.
        self.pump_until(lambda: self.window.remote_active)

    def test_remote_priority_lease_overrides_gui_without_zero_and_recovers(self):
        self.begin()
        self.extra = Node("physical_remote")
        pub = self.extra.create_publisher(LocoVelocity, self.settings["topic"], qos_profile_sensor_data)
        msg = LocoVelocity()
        msg.priority = LocoVelocity.PRIORITY_REMOTE
        msg.linear_vx, msg.keep_time = 0.6, 0.2
        for _ in range(6):
            pub.publish(msg)
            self.pump(0.04)
        self.assertTrue(self.window.remote_active)
        self.assertFalse(self.window.sending)
        self.messages.clear()
        for _ in range(3):
            pub.publish(msg)
            self.pump(0.04)
        self.assertFalse(any(m.priority == LocoVelocity.PRIORITY_NORMAL for m in self.messages))
        self.pump_until(lambda: self.window.can_send())
        # Driver still exists: GUI can be operated anew after its lease expires.
        self.assertFalse(self.window.sending)
        self.assertTrue(all(box.value() == 0 for box in self.window.axes))
        self.window.axes[0].setValue(0.2)
        self.pump_until(lambda: self.window.sending)

    def test_remote_takeover_and_release_in_one_batch_never_restores_gui_values(self):
        self.begin()
        msg = LocoVelocity()
        msg.priority, msg.keep_time = LocoVelocity.PRIORITY_REMOTE, 0.2
        self.window.observe_velocity(msg)
        msg.keep_time = 0.0
        self.window.observe_velocity(msg)
        self.pump(0.1)
        self.assertFalse(self.window.remote_active)
        self.assertFalse(self.window.sending)
        self.assertTrue(all(box.value() == 0 for box in self.window.axes))

    def test_offline_action_service_accept_reject_and_no_duplicate(self):
        self.begin()
        self.pump_until(lambda: self.window.action_trigger.isEnabled())
        self.window.action_select.setCurrentIndex(2)
        self.window.request_action()
        self.window.request_action()
        self.pump_until(lambda: self.window.motion is None)
        self.assertEqual(self.motion_commands, [(13, "")])
        self.assertIn("已接受", self.window.action_result.text())
        self.assertFalse(self.window.sending)
        self.motion_reject = True
        self.window.request_action()
        self.pump_until(lambda: self.window.motion is None)
        self.assertIn("被拒绝", self.window.action_result.text())
        self.assertIn("3", self.window.action_result.text())

    def test_offline_actions_follow_controller_motion_manifest(self):
        with tempfile.TemporaryDirectory() as folder:
            config = yaml.safe_load((ROOT / "tests/data/controller.yaml").read_text())
            config["policies"] = {
                mode: str((ROOT / "tests/data" / profile).resolve())
                for mode, profile in config["policies"].items()}
            config["motion_stream"] = "motions.yaml"
            source = Path(folder) / "controller.yaml"
            source.write_text(yaml.safe_dump(config))
            (Path(folder) / "motions.yaml").write_text(yaml.safe_dump({
                "service": "/configured_motion",
                "motions": [{"cmd": 11, "name": "奥特曼", "file": "a.csv"},
                            {"cmd": 14, "name": "再见", "file": "b.csv"}]}))
            settings = gui.load_settings(source, ROOT / "config/remote_gui.yaml")
            self.assertEqual(settings["motion_cmd_service"], "/configured_motion")
            self.assertEqual(settings["offline_actions"], [
                {"cmd": 11, "name": "JOF11 · 奥特曼"},
                {"cmd": 14, "name": "JOF14 · 再见"}])
            self.assertEqual(settings["motion_mode"], 28)

    def test_configured_amp_actions_reject_other_modes_including_xbox(self):
        self.settings["motion_mode"] = 28
        for mode in (1, 2, 30, 41):
            self.mode = mode
            self.pump_until(lambda: self.window.mode == mode)
            self.window.refresh_controls()
            self.assertFalse(self.window.action_trigger.isEnabled())
            self.window.request_action()
            self.window.request_action(11, from_xbox=True)
            self.assertIsNone(self.window.motion)
            self.assertIn("AMP WALK", self.window.action_result.text())
        self.assertEqual(self.motion_commands, [])
        self.mode = 28
        self.pump_until(lambda: self.window.action_trigger.isEnabled())
        self.window.request_action()
        self.pump_until(lambda: self.window.motion is None)
        self.assertEqual(self.motion_commands, [(11, "")])

    def test_offline_timeout_is_not_retried_or_automatically_resumed(self):
        self.begin()
        self.window.request_action()
        self.drop = True
        self.pump_until(lambda: self.window.motion is None)
        self.assertFalse(self.window.sending)
        self.assertIn("不自动重试", self.window.action_result.text())
        self.drop = False
        self.pump(0.3)
        self.assertLessEqual(len(self.motion_commands), 1)

    def test_offline_action_missing_server_and_still_preemption(self):
        self.server.destroy_service(self.motion_service)
        self.pump_until(lambda: not self.window.motion_client.service_is_ready())
        self.window.request_action()
        self.assertIsNone(self.window.motion)
        self.assertIn("未连接", self.window.action_result.text())
        self.motion_service = self.server.create_service(
            MotionCmd, self.settings["motion_cmd_service"], self.handle_motion)
        self.pump_until(lambda: self.window.action_trigger.isEnabled())
        self.window.request_action()
        self.window.request_mode(1)
        self.assertIsNone(self.window.motion)
        self.pump_until(lambda: not self.window.mode_requests and self.window.mode == 1)

    def test_remote_takeover_discards_action_wait_without_retry(self):
        self.window.request_action()
        self.assertIsNotNone(self.window.motion)
        with tempfile.TemporaryDirectory() as folder:
            device = Path(folder) / "js0"
            self.settings["remote_device_globs"] = [str(device)]
            device.touch()
            self.window.sync_remote()
            self.assertIsNone(self.window.motion)
            self.assertIn("可能已执行", self.window.action_result.text())
            self.pump(0.1)
            self.assertLessEqual(len(self.motion_commands), 1)

    def test_fault_prevents_offline_action_request(self):
        self.fault = True
        self.pump_until(lambda: self.window.fault)
        self.window.request_action()
        self.assertIsNone(self.window.motion)
        self.assertFalse(self.window.action_trigger.isEnabled())
        self.assertEqual(self.motion_commands, [])

    def test_still_supersedes_pending_mode_request(self):
        self.begin()
        self.drop = True
        self.window.request_mode(30)
        self.window.request_mode(1)
        self.assertTrue(self.window.mode_requests)
        self.drop = False
        self.pump_until(lambda: not self.window.mode_requests and self.window.mode == 1)
        self.assertIn(1, self.commands)
        self.assert_zero()

    def test_repeated_still_sends_and_retains_every_response(self):
        self.begin()
        self.drop = True
        self.window.request_mode(30)
        for _ in range(30):
            self.window.request_mode(1)
        pending = self.window.mode_requests[:]
        self.assertEqual(len(pending), 31)
        self.assertFalse(self.window.sending)
        self.drop = False
        self.pump_until(lambda: not self.window.mode_requests and self.window.mode == 1)
        self.assertEqual(self.commands.count(1), 30)
        self.assertEqual(self.commands.count(30), 1)
        self.assertTrue(all(future.done() and not future.cancelled()
                            and future.result().result for _, _, (future, _) in pending))

    def test_mode_responses_out_of_order_are_all_accounted_for(self):
        futures = [Future(), Future()]
        with patch.object(self.window, "call", side_effect=[(f, time.monotonic()) for f in futures]):
            self.window.request_mode(1)
            self.window.request_mode(1)
        futures[1].set_result(LocomotionModeSwitch.Response(result=False, mode=1, fb_msg="busy"))
        self.window.read_mode_responses()
        self.assertEqual(len(self.window.mode_requests), 1)
        self.assertIn("#2", self.window.command_result.text())
        self.assertIn("busy", self.window.command_result.text())
        futures[0].set_result(LocomotionModeSwitch.Response(result=True, mode=1, fb_msg="STILL"))
        self.window.read_mode_responses()
        self.assertFalse(self.window.mode_requests)
        self.assertIn("#1", self.window.command_result.text())

    def test_one_mode_timeout_does_not_discard_later_still(self):
        futures = [Future(), Future()]
        now = time.monotonic()
        with patch.object(self.window, "call", side_effect=[
                (futures[0], now - self.settings["service_timeout"] - 0.1), (futures[1], now)]):
            self.window.request_mode(1)
            self.window.request_mode(1)
        self.window.read_mode_responses()
        self.assertTrue(futures[0].cancelled())
        self.assertFalse(futures[1].done())
        self.assertEqual(len(self.window.mode_requests), 1)
        self.assertIn("超时", self.window.command_result.text())
        futures[1].set_result(LocomotionModeSwitch.Response(result=True, mode=1, fb_msg="STILL"))
        self.window.read_mode_responses()
        self.assertFalse(self.window.mode_requests)
        self.assertIn("#2", self.window.command_result.text())

    def test_pending_mode_blocks_other_motion_requests(self):
        self.window.request_mode(30)
        for _ in range(20):
            self.window.request_mode(2)
            self.window.request_action()
        self.pump_until(lambda: not self.window.mode_requests)
        self.assertEqual([cmd for cmd in self.commands if cmd != 0], [30])
        self.assertEqual(self.motion_commands, [])

    def test_service_clients_retain_batched_replies(self):
        # Exercise the actual GUI clients at the transport boundary. Pausing
        # client dispatch models a busy Qt loop; it is not normal GUI concurrency.
        self.window.discard(self.window.query)
        self.window.query = None
        for client, service_type, cmd, received in (
                (self.window.client, LocomotionModeSwitch, 0, self.commands),
                (self.window.motion_client, MotionCmd, 11, self.motion_commands)):
            with self.subTest(service=service_type.__name__):
                futures = []
                try:
                    for _ in range(14):
                        before = len(received)
                        futures.append(client.call_async(service_type.Request(cmd=cmd)))
                        deadline = time.monotonic() + 2
                        while len(received) == before and time.monotonic() < deadline:
                            self.executor.spin_once(timeout_sec=0.01)
                        self.assertGreater(len(received), before, "server did not receive request")
                    time.sleep(0.05)  # Allow DDS delivery without taking responses.
                    deadline = time.monotonic() + 1
                    while not all(f.done() for f in futures) and time.monotonic() < deadline:
                        rclpy.spin_once(self.node, timeout_sec=0.01)
                    self.assertEqual(sum(f.done() for f in futures), 14,
                                     "replies lost while client executor was paused")
                    self.assertTrue(all(f.result().result for f in futures))
                finally:
                    for future in futures:
                        if not future.done():
                            client.remove_pending_request(future)
                            future.cancel()

    def test_settings_validation(self):
        default = gui.load_settings(ROOT / "tests/data/controller.yaml", ROOT / "config/remote_gui.yaml")
        with tempfile.TemporaryDirectory() as directory:
            settings_path = Path(directory) / "gui.yaml"
            for change in ({"keep_time": 10}, {"publish_hz": float("nan")},
                           {"keep_time": 0.001}, {"status_timeout": 0.01},
                           {"auto_center": "false"}, {"open_depth_viewer": "true"},
                           {"velocity_deadband": 1}, {"velocity_deadband": float("nan")},
                           {"zero_stop_seconds": -1}, {"remote_device_globs": ["js0"]},
                           {"remote_publisher_nodes": ["driver"]}, {"motion_cmd_service": ""},
                           {"offline_actions": [{"name": "bad", "cmd": 1}]},
                           {"offline_actions": [{"name": "bad", "cmd": True}]},
                           {"offline_actions": [{"name": "a", "cmd": 11}, {"name": "b", "cmd": 11}]}):
                invalid = copy.copy(default)
                invalid.update(change)
                settings_path.write_text(yaml.safe_dump(invalid))
                with self.assertRaises(ValueError):
                    gui.load_settings(ROOT / "tests/data/controller.yaml", settings_path)
            config_path = Path(directory) / "sbus.yaml"
            config = yaml.safe_load((ROOT / "tests/data/controller.yaml").read_text())
            config["remote"]["source"] = "sbus"
            config["policies"] = {k: str(ROOT / "tests/data" / v) for k, v in config["policies"].items()}
            config_path.write_text(yaml.safe_dump(config))
            self.assertEqual(gui.load_settings(config_path, ROOT / "config/remote_gui.yaml")["remote_source"], "sbus")

    def test_perception_only_config_and_missing_velocity_policy(self):
        config = yaml.safe_load((ROOT / "tests/data/controller.yaml").read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "controller.yaml"
            config["policies"] = {41: str(ROOT / "tests/data/velocity.yaml")}
            path.write_text(yaml.safe_dump(config))
            settings = gui.load_settings(path, ROOT / "config/remote_gui.yaml")
            self.assertEqual(settings["modes"], [1, 2, 41])
            self.assertEqual(settings["low"], self.settings["low"])
            self.assertEqual(settings["high"], self.settings["high"])
            for policies in ({}, {30: str(ROOT / "tests/data/velocity.yaml")}):
                config["policies"] = policies
                path.write_text(yaml.safe_dump(config))
                with self.assertRaisesRegex(ValueError, "缺少"):
                    gui.load_settings(path, ROOT / "config/remote_gui.yaml")

    def test_special_mimic_modes_follow_enabled_controller_profiles(self):
        config = yaml.safe_load((ROOT / "tests/data/controller.yaml").read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "controller.yaml"
            modes = [28, 35, 36, 37, 38, 42, 43]
            config["policies"] = {mode: str(ROOT / "tests/data/velocity.yaml") for mode in modes}
            path.write_text(yaml.safe_dump(config))
            settings = gui.load_settings(path, ROOT / "config/remote_gui.yaml")
            self.assertEqual(settings["modes"], [1, 2] + modes)
            for mode in modes:
                self.assertIn(mode, gui.MODE_NAMES)
            config["policies"].pop(38)
            path.write_text(yaml.safe_dump(config))
            self.assertNotIn(38, gui.load_settings(path, ROOT / "config/remote_gui.yaml")["modes"])
            config["robot"] = "robot.yaml"
            path.write_text(yaml.safe_dump(config))
            for identity in ("pro", "sport"):
                (path.parent / "robot.yaml").write_text(yaml.safe_dump({"identity": identity}))
                self.assertEqual(gui.load_settings(path, ROOT / "config/remote_gui.yaml")["robot_identity"],
                                 identity.upper())

    def test_parent_exit_closes_window(self):
        self.begin()
        self.window.parent_pid = -1
        self.window.tick()
        self.assertTrue(self.window.closed)
        self.assert_zero()

    def test_deadband_zero_tail_and_automatic_restart(self):
        for box in self.window.axes:
            box.setValue(0.01)
        self.pump(0.1)
        self.assertFalse(self.window.sending)
        self.assertEqual(self.messages, [])
        self.begin()
        self.window.zero_velocity()
        self.assert_zero(stopped=False)
        self.pump(0.2)
        self.assertTrue(self.window.sending)
        self.window.zero_since = time.monotonic() - 5.1
        self.pump(0.1)
        self.assertFalse(self.window.sending)
        count = len(self.messages)
        self.pump(0.15)
        self.assertEqual(len(self.messages), count)
        self.window.axes[2].setValue(0.3)
        self.pump_until(lambda: self.window.sending)
        self.pump_until(lambda: self.messages[-1].angular_wz > 0.2)
        self.assertAlmostEqual(self.messages[-1].keep_time, 0.2)

    def test_zero_tail_lasts_five_real_seconds(self):
        self.begin()
        self.messages.clear()
        started = time.monotonic()
        self.window.zero_velocity()
        self.pump(0.1)  # Drain velocity already in flight before the zero command.
        self.messages.clear()
        self.pump(4.7)
        self.assertTrue(self.window.sending)
        self.assertTrue(self.messages)
        self.assertTrue(all(message.linear_vx == message.linear_vy == message.angular_wz == 0
                            for message in self.messages))
        self.pump_until(lambda: not self.window.sending, timeout=1)
        self.assertGreaterEqual(time.monotonic() - started, 5)

    def test_depth_viewer_waits_for_applied_mode_and_closes(self):
        self.settings["open_depth_viewer"] = True
        self.settings["depth_image_topic"] = "/gui_test/depth"
        child = Mock(returncode=None)
        child.poll.side_effect = lambda: child.returncode
        child.terminate.side_effect = lambda: setattr(child, "returncode", 0)
        with patch.object(gui.subprocess, "Popen", return_value=child) as launch:
            self.reject = True
            self.window.request_mode(41)
            self.pump_until(lambda: not self.window.mode_requests)
            self.pump(0.1)
            launch.assert_not_called()
            self.reject = False
            self.window.request_mode(41)
            self.pump_until(lambda: self.window.depth_process is not None)
            self.pump(0.4)
            self.assertEqual(launch.call_count, 1)
            self.assertIn("/gui_test/depth", launch.call_args.args[0])
            self.assertTrue(launch.call_args.kwargs["start_new_session"])
            self.mode = 28
            self.pump_until(lambda: self.window.depth_process is None)
            child.terminate.assert_called_once()
            child.returncode = None
            self.mode = 41
            self.pump_until(lambda: launch.call_count == 2)
            self.window.close()
            self.assertEqual(child.terminate.call_count, 2)


if __name__ == "__main__":
    unittest.main()
