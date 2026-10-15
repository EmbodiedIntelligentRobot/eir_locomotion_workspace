"""External CSV GUI against the workspace's real headless MuJoCo controller."""
import argparse
import copy
import csv
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import remote_gui as gui
from motion_stream_tools.csv_codec import MotionFrame, write_motion_csv

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--controller', required=True)
parser.add_argument('--config', required=True)
args = parser.parse_args()
source = Path(args.config).resolve()
config = gui.yaml.safe_load(source.read_text())
assert config['runtime']['backend'] == 'simulation', 'Only simulation configs are allowed'
for key in ('robot', 'motion_stream'):
    config[key] = str((source.parent / config[key]).resolve())
config['policies'] = {k: str((source.parent / v).resolve()) for k, v in config['policies'].items()}
config['policies'].pop(41, None)
config['runtime'].update(headless=True, model=str((source.parent / os.path.expandvars(config['runtime']['model'])).resolve()))
config['runtime'].pop('depth', None)
config.get('debug', {}).pop('depth_image_topic', None)
config['remote']['gui'] = False
motion_source = Path(config['motion_stream'])
original_motions = motion_source.read_bytes()
app = gui.QtWidgets.QApplication([])
gui.rclpy.init(args=[])
node = gui.Node('external_csv_gui_check')
with tempfile.TemporaryDirectory(prefix='external_gui_check_') as directory, tempfile.TemporaryFile(mode='w+') as log:
    root = Path(directory)
    path = root / 'controller.yaml'
    path.write_text(gui.yaml.safe_dump(config))
    settings = gui.load_settings(path, Path(gui.__file__).parent / 'config/remote_gui.yaml')
    settings.update(xbox_enabled=False, open_depth_viewer=False, remote_device_globs=[])
    original_actions = copy.deepcopy(settings['offline_actions'])
    window = gui.RemoteWindow(node, settings)
    process = subprocess.Popen([args.controller, '--no-gui', str(path)], stdout=log, stderr=subprocess.STDOUT)
    def wait(predicate, seconds=12):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert process.poll() is None, 'Controller exited unexpectedly'
            app.processEvents()
            if predicate(): return
            time.sleep(.01)
        raise AssertionError(window.file_result.text()+'\n'+window.detail.toPlainText())
    try:
        wait(lambda: window.healthy() and window.mode == 1, 40)
        window.request_mode(2)
        wait(lambda: window.mode == 2 and 'ready=true' in window.detail.toPlainText() and not window.mode_requests, 25)
        window.request_mode(28)
        wait(lambda: window.mode == 28 and not window.mode_requests)
        print('PASS real workspace controller connects to external GUI in AMP', flush=True)
        positions = [0.] * 44
        positions[14] = .1
        stream = root / 'upper.csv'
        write_motion_csv(stream, [MotionFrame(i*.02, positions, joint_pos_mask=1 << 14) for i in range(201)], 44)
        original_csv = stream.read_bytes()
        mime = gui.QtCore.QMimeData(); mime.setUrls([gui.QtCore.QUrl.fromLocalFile(str(stream))])
        drop = gui.QtGui.QDropEvent(gui.QtCore.QPointF(0,0), gui.QtCore.Qt.CopyAction, mime,
                                    gui.QtCore.Qt.LeftButton, gui.QtCore.Qt.NoModifier)
        window.motion_file.dropEvent(drop)
        assert drop.isAccepted() and window.motion_file.text() == str(stream)
        window.file_player.start()
        wait(lambda: window.file_player.phase == 'native_playing' and 'upper_motion_cmd=255' in window.detail.toPlainText())
        assert not window.can_send() and not window.action_trigger.isEnabled()
        assert window.file_player.publisher is None
        assert process.poll() is None and motion_source.read_bytes() == original_motions
        window.file_player.stop('用户停止')
        wait(lambda: not window.file_player.active)
        assert window.file_player.temporary is None
        print('PASS CSV drop, native playback, safety gating and stop; no restart or topic dependency', flush=True)
        write_motion_csv(stream, [MotionFrame(i*.02, positions, joint_pos_mask=1 << 14) for i in range(11)], 44)
        window.select_motion_file(str(stream)); window.file_player.start()
        wait(lambda: not window.file_player.active)
        assert '结束' in window.file_result.text(), window.file_result.text()
        print('PASS short file completes and releases GUI control', flush=True)
        positions[14] = 100
        write_motion_csv(stream, [MotionFrame(i*.02, positions, joint_pos_mask=1 << 14) for i in range(11)], 44)
        window.select_motion_file(str(stream)); window.file_player.start()
        wait(lambda: not window.file_player.active)
        assert '拒绝' in window.file_result.text()
        assert settings['offline_actions'] == original_actions and motion_source.read_bytes() == original_motions
        print('PASS controller rejects out-of-limit file; numbered actions and original resources unchanged', flush=True)
        stream.write_bytes(original_csv)
        window.select_motion_file(str(stream)); window.file_player.start()
        wait(lambda: window.file_player.phase == 'native_playing')
        window.request_mode(1)
        wait(lambda: not window.file_player.active and not window.mode_requests and window.mode == 1)
        assert stream.read_bytes() == original_csv
        print('PASS STILL stops CSV and source CSV remains unchanged', flush=True)
    except Exception:
        log.seek(0); print(log.read()[-6000:]); raise
    finally:
        window.close(); node.destroy_node(); gui.rclpy.shutdown()
        process.terminate()
        try: process.wait(timeout=8)
        except subprocess.TimeoutExpired: process.kill(); process.wait()
