# SPDX-License-Identifier: BSD-3-Clause
"""Nonblocking Linux joystick reader. No ROS or robot commands in this module."""
import array
import fcntl
import os
import struct
import time


class Xbox:
    # Linux input codes, not device-dependent joystick indices.
    LX, LY, LT, RX, RY, RT, HAT_X, HAT_Y = 0, 1, 2, 3, 4, 5, 16, 17
    BUTTONS = {304: "A", 305: "B", 307: "X", 308: "Y", 310: "LB",
               311: "RB", 314: "BACK", 315: "START"}
    # Humanoid shortcut order, with START/BACK exchanged at user request.
    # AMPWALK maps to this controller's AMP 28.
    SHORTCUTS = (
        ({"LT", "RT"}, 1), ({"LB", "RB"}, 253),
        ({"BACK", "B"}, 2), ({"BACK", "Y"}, 10),
        ({"BACK", "X"}, 22), ({"BACK", "A"}, 28),
        ({"BACK", "UP"}, 30), ({"BACK", "DOWN"}, 31),
        ({"BACK", "LEFT"}, 32), ({"BACK", "RIGHT"}, 33),
        ({"START", "LEFT"}, 34), ({"START", "RIGHT"}, 39),
        ({"BACK", "LT"}, 35), ({"BACK", "RT"}, 36),
        ({"BACK", "LB"}, 37), ({"BACK", "RB"}, 38))
    SINGLE_PRESSES = {"UP": 11, "DOWN": 12, "LEFT": 13, "RIGHT": 14}

    EQR_SHORTCUTS = (({"LT", "RT"}, 1), ({"BACK", "B"}, 2),
                     ({"BACK", "A"}, 103))

    def __init__(self, quadruped=False):
        if quadruped:
            # Other RL modes are selected explicitly in the GUI; no humanoid actions.
            self.SHORTCUTS = self.EQR_SHORTCUTS
            self.SINGLE_PRESSES = {}
        self.fd = None
        self.path = ""
        self.error = ""
        self.retry_at = 0.0
        self.axes, self.buttons = {}, {}
        self.disarm()
        self.previous_combos = set()

    @property
    def connected(self):
        return self.fd is not None

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
        self.fd, self.path = None, ""
        self.axes, self.buttons = {}, {}
        self.disarm()
        self.previous_combos = set()

    def open(self, path):
        self.close()
        try:
            self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            count_axes, count_buttons = array.array('B', [0]), array.array('B', [0])
            fcntl.ioctl(self.fd, 0x80016a11, count_axes)  # JSIOCGAXES
            fcntl.ioctl(self.fd, 0x80016a12, count_buttons)  # JSIOCGBUTTONS
            axes, buttons = array.array('B', [0] * 64), array.array('H', [0] * 512)
            fcntl.ioctl(self.fd, 0x80406a32, axes)  # JSIOCGAXMAP
            fcntl.ioctl(self.fd, 0x84006a34, buttons)  # JSIOCGBTNMAP
            self.axis_map = list(axes[:count_axes[0]])
            self.button_map = list(buttons[:count_buttons[0]])
            if not {self.LX, self.LY, self.LT, self.RX, self.RY, self.RT,
                    self.HAT_X, self.HAT_Y} <= set(self.axis_map) or not set(self.BUTTONS) <= set(self.button_map):
                raise OSError("不支持此手柄映射；F710 请切至 X 模式")
            self.path, self.error = path, ""
        except OSError:
            self.close()
            raise

    def neutral(self, deadband):
        return all(code in self.axes and abs(self.axes[code]) <= deadband
                   for code in (self.LX, self.LY, self.RX, self.RY))

    def disarm(self):
        self.armed = False
        self.click_candidate = None
        self.click_blocked = True

    def pressed(self):
        keys = {self.BUTTONS.get(code, str(code)) for code, value in self.buttons.items() if value}
        for code, name in ((self.LT, "LT"), (self.RT, "RT")):
            # Reference driver: raw > -32768 + 2000 * 10.
            if self.axes.get(code, -1.0) > -12768 / 32767:
                keys.add(name)
        for code, negative, positive in ((self.HAT_X, "LEFT", "RIGHT"),
                                          (self.HAT_Y, "UP", "DOWN")):
            value = self.axes.get(code, 0.0)
            if abs(value) > 2000 / 32767:
                keys.add(negative if value < 0 else positive)
        return keys

    def actions(self, deadband, initial):
        keys = self.pressed()
        combos = {i for i, (buttons, _) in enumerate(self.SHORTCUTS) if buttons <= keys}
        new = combos - self.previous_combos
        self.previous_combos = combos
        if initial:
            self.disarm()
            return []
        if new:
            index = min(new)
            self.click_candidate, self.click_blocked = None, True
            if self.armed or index == 0:  # Explicit STILL remains available while blocked.
                return [("mode", self.SHORTCUTS[index][1])]
        if not self.armed:
            return []
        neutral = self.neutral(deadband) and not keys
        if self.click_blocked:
            if neutral:
                self.click_blocked = False
            return []
        if self.click_candidate is not None:
            candidate = self.click_candidate
            if keys == {candidate} and self.neutral(deadband):
                return []
            self.click_candidate = None
            if neutral:
                return [("motion", self.SINGLE_PRESSES[candidate])]
            self.click_blocked = True
        elif keys:
            if len(keys) == 1 and next(iter(keys)) in self.SINGLE_PRESSES and self.neutral(deadband):
                self.click_candidate = next(iter(keys))
            else:
                self.click_blocked = True
        return []

    def poll(self, paths, deadband):
        """Return combo edges or standalone D-pad releases; ignore initialization."""
        if self.connected and self.path not in paths:
            self.close()
        if not self.connected:
            if time.monotonic() < self.retry_at:
                return []
            self.retry_at = time.monotonic() + 1.0
            self.error = ""
            for path in paths:
                try:
                    self.open(path)
                    break
                except OSError as error:
                    self.error = f"手柄无法读取：{error}"
            if not self.connected:
                return []
        actions = []
        try:
            # Drain current state before arming. Bound work to keep Qt responsive.
            for _ in range(256):
                try:
                    event = os.read(self.fd, 8)
                except BlockingIOError:
                    break
                if len(event) != 8:
                    raise OSError("手柄已断开")
                _, value, kind, index = struct.unpack('IhBB', event)
                initial = bool(kind & 0x80)
                kind &= 0x7f
                if kind == 2 and index < len(self.axis_map):
                    self.axes[self.axis_map[index]] = max(-1.0, value / 32767.0)
                elif kind == 1 and index < len(self.button_map):
                    code = self.button_map[index]
                    self.buttons[code] = bool(value)
                actions.extend(self.actions(deadband, initial))
            else:
                self.disarm()
                # Drop queued motion, but keep an already observed explicit stop.
                return [("mode", 1)] if ("mode", 1) in actions else []
        except OSError as error:
            self.close()
            self.error = str(error)
            return []
        if (not self.armed and self.neutral(deadband)
                and all(code in self.buttons for code in self.BUTTONS)
                and all(code in self.axes for code in self.axis_map)
                and not self.pressed()):
            self.armed = True
            self.click_blocked = False
        # Only one command per batch; a mode combo suppresses all single clicks.
        modes = [cmd for kind, cmd in actions if kind == "mode"]
        if modes:
            order = [cmd for _, cmd in self.SHORTCUTS]
            return [("mode", min(modes, key=order.index))]
        return actions[:1]

    def velocity(self, low, high, deadband):
        if not self.armed:
            return (0.0, 0.0, 0.0)
        values = [-self.axes.get(code, 0.0) for code in (self.LY, self.LX, self.RX)]
        return tuple(0.0 if abs(value) <= deadband else
                     value * (hi if value > 0 else -lo)
                     for value, lo, hi in zip(values, low, high))
