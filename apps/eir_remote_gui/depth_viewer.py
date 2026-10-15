#!/usr/bin/python3
# SPDX-License-Identifier: BSD-3-Clause
"""Own one rqt process and close it when the remote GUI exits, including SIGKILL."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading

from ament_index_python.packages import get_package_prefix
from PyQt5.QtCore import QSettings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--parent-pid", required=True, type=int)
    args = parser.parse_args()
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    if os.getppid() != args.parent_pid or stop.is_set():
        return 0
    executable = Path(get_package_prefix("rqt_image_view")) / "lib/rqt_image_view/rqt_image_view"
    # Personal rqt settings can retain a 10 m scale, making nearby policy
    # depth almost black. Keep display settings local to this owned viewer.
    config = tempfile.TemporaryDirectory(prefix="eir-depth-viewer-")
    settings_path = Path(config.name) / "ros.org/rqt_gui.ini"
    settings_path.parent.mkdir()
    settings = QSettings(str(settings_path), QSettings.IniFormat)
    plugin = "perspective/@rqt_image_view__ImageView/pluginmanager/plugin__rqt_image_view__ImageView__0/plugin/"
    settings.setValue(plugin + "dynamic_range", True)
    settings.sync()
    environment = dict(os.environ, XDG_CONFIG_HOME=config.name)
    process = None
    try:
        process = subprocess.Popen(["/usr/bin/python3", str(executable), "--force-discover", args.topic],
                                   stdin=subprocess.DEVNULL, env=environment)
        while os.getppid() == args.parent_pid and not stop.wait(0.1):
            if process.poll() is not None:
                return process.returncode
        return 0
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        config.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
