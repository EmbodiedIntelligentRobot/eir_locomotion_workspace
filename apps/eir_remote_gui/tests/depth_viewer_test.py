#!/usr/bin/python3
# SPDX-License-Identifier: BSD-3-Clause
"""Real rqt + synthetic depth test, isolated DDS and offscreen Qt; no robot/camera."""
import os
from pathlib import Path
import struct
import subprocess
import tempfile
import time
import yaml

from remote_gui_test import RemoteGuiTest, ROOT
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from PyQt5.QtCore import QSettings


def active(pid):
    try:
        return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0] != "Z"
    except FileNotFoundError:
        return False


def child_of(pid):
    try:
        values = Path(f"/proc/{pid}/task/{pid}/children").read_text().split()
        return int(values[0]) if values else None
    except FileNotFoundError:
        return None


def main():
    RemoteGuiTest.setUpClass()
    test = RemoteGuiTest(methodName="runTest")
    test.setUp()
    standalone = None
    try:
        with tempfile.TemporaryDirectory(prefix="eir-rqt-test-") as directory:
            previous_config_home = os.environ.get("XDG_CONFIG_HOME")
            os.environ["XDG_CONFIG_HOME"] = directory
            try:
                # A user's saved 10 m range must not darken the policy viewer.
                personal_path = Path(directory) / "ros.org/rqt_gui.ini"
                personal_path.parent.mkdir()
                personal = QSettings(str(personal_path), QSettings.IniFormat)
                plugin = "perspective/@rqt_image_view__ImageView/pluginmanager/plugin__rqt_image_view__ImageView__0/plugin/"
                personal.setValue(plugin + "dynamic_range", False)
                personal.setValue(plugin + "max_range", 10)
                personal.sync()
                personal_content = personal_path.read_bytes()
                topic = "/gui_test/depth"
                test.settings.update(open_depth_viewer=True, depth_image_topic=topic)
                publisher = test.server.create_publisher(Image, topic, qos_profile_sensor_data)
                message = Image(height=18, width=32, encoding="32FC1", step=128)
                message.data = struct.pack("<576f", *[i / 576 for i in range(576)])
                count = [0]
                def publish():
                    message.header.stamp = test.server.get_clock().now().to_msg()
                    publisher.publish(message)
                    count[0] += 1
                test.server.create_timer(0.05, publish)
                def endpoints():
                    return {tuple(info.endpoint_gid) for info in
                            test.server.get_subscriptions_info_by_topic(topic)}
                def enter():
                    before = endpoints()
                    test.mode = 41
                    test.pump_until(lambda: test.window.depth_process is not None, timeout=5)
                    owner = test.window.depth_process.pid
                    test.pump_until(lambda: child_of(owner) is not None, timeout=10)
                    viewer = child_of(owner)
                    test.pump_until(lambda: bool(endpoints() - before), timeout=20)
                    assert all(info.qos_profile.reliability == qos_profile_sensor_data.reliability
                               for info in test.server.get_subscriptions_info_by_topic(topic)
                               if tuple(info.endpoint_gid) not in before), "rqt incompatible with best-effort depth"
                    test.pump(0.5)
                    assert active(viewer)
                    environment = dict(entry.split(b"=", 1) for entry in
                        Path(f"/proc/{viewer}/environ").read_bytes().split(b"\0") if b"=" in entry)
                    private_config = Path(os.fsdecode(environment[b"XDG_CONFIG_HOME"]))
                    assert private_config != Path(directory), "rqt inherited personal display settings"
                    display_settings = QSettings(str(private_config / "ros.org/rqt_gui.ini"), QSettings.IniFormat)
                    assert display_settings.value(plugin + "dynamic_range", False, type=bool)
                    assert personal_path.read_bytes() == personal_content, "personal rqt settings changed"
                    return owner, viewer, private_config
                owner, viewer, private_config = enter()
                print(f"PASS actual rqt subscribed to {topic}, synthetic 32FC1 frames={count[0]}", flush=True)
                test.mode = 28
                test.pump_until(lambda: test.window.depth_process is None, timeout=6)
                test.pump_until(lambda: not active(viewer) and not active(owner), timeout=5)
                assert not private_config.exists(), "temporary viewer settings were not removed"
                print("PASS leaving perception closed owned rqt", flush=True)
                owner, viewer, private_config = enter()
                test.window.close()
                test.pump_until(lambda: not active(viewer) and not active(owner), timeout=5)
                assert not private_config.exists(), "temporary viewer settings were not removed"
                print("PASS closing remote GUI closed owned rqt", flush=True)
                # A separate real GUI must also clean up rqt if killed without closeEvent.
                config = yaml.safe_load((ROOT / "tests/data/controller.yaml").read_text())
                config["remote"].update(velocity_topic=test.settings["topic"],
                                        mode_service=test.settings["service"])
                config["policies"] = {key: str(ROOT / "tests/data" / value)
                                      for key, value in config["policies"].items()}
                config_path = Path(directory) / "controller.yaml"
                config_path.write_text(yaml.safe_dump(config))
                gui_path = Path(directory) / "gui.yaml"
                settings = yaml.safe_load((ROOT / "config/remote_gui.yaml").read_text())
                settings["depth_image_topic"] = topic
                gui_path.write_text(yaml.safe_dump(settings))
                standalone = subprocess.Popen(["/usr/bin/python3", str(ROOT / "remote_gui.py"),
                    "--config", str(config_path), "--gui-config", str(gui_path)])
                before = endpoints()
                test.pump_until(lambda: child_of(standalone.pid) is not None, timeout=10)
                owner = child_of(standalone.pid)
                test.pump_until(lambda: child_of(owner) is not None, timeout=10)
                viewer = child_of(owner)
                test.pump_until(lambda: bool(endpoints() - before), timeout=20)
                standalone.kill()
                standalone.wait(timeout=5)
                test.pump_until(lambda: not active(viewer) and not active(owner), timeout=6)
                print("PASS SIGKILL of remote GUI left no live rqt/supervisor", flush=True)
            finally:
                if previous_config_home is None:
                    os.environ.pop("XDG_CONFIG_HOME", None)
                else:
                    os.environ["XDG_CONFIG_HOME"] = previous_config_home
    finally:
        if standalone is not None and standalone.poll() is None:
            standalone.terminate()
            standalone.wait(timeout=5)
        test.tearDown()
        RemoteGuiTest.tearDownClass()


if __name__ == "__main__":
    main()
