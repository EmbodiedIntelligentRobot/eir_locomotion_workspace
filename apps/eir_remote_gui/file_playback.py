"""Local CSV playback through the existing topic interface; never writes offline files."""
from bisect import bisect_right
import time
import csv
import math
from pathlib import Path
import tempfile
import uuid
import re
import yaml

from rclpy.qos import qos_profile_sensor_data


def load_playback_file(path):
    """Use the Motion Stream project's importers; conversion stays in temporary storage."""
    from remote_gui import check_motion_stream_file
    from motion_stream_tools.csv_codec import read_motion_csv
    from motion_stream_tools.importers import convert_path, read_table
    from motion_stream_tools.layout import load_any_layout
    from motion_stream_tools.motion_editor_importer import (
        is_aiq11_motion_editor_csv, convert_aiq11_motion_editor_csv)

    source = Path(path).expanduser().resolve(strict=True)
    if not source.is_file() or source.suffix.lower() != '.csv':
        raise ValueError('请选择 CSV 文件')
    with source.open(encoding='utf-8-sig', newline='') as stream:
        header = next(csv.reader(stream), [])
    if any(name.startswith('joint.pos[') for name in header):
        check_motion_stream_file(source)
        return source, read_motion_csv(source, 44, strict=True), False
    layout = load_any_layout('aiq11pro')
    compact = is_aiq11_motion_editor_csv(source)
    if not compact:
        table = read_table(source)
        required = [f'{side}_leg_{i}' for side in ('l', 'r') for i in range(1, 7)]
        required += [f'torso_{i}' for i in range(1, 4)]
        required += [f'{side}_arm_{i}' for side in ('l', 'r') for i in range(1, 5)]
        if not all(name in table.columns for name in required) or not any(
                name in table.columns for name in ('timestamp', 'time')):
            raise ValueError('原始 CSV 需包含时间戳、双腿、躯干和双臂关节列')
        if not all(math.isfinite(float(value)) for row in table.itertuples(index=False, name=None)
                   for value in row):
            raise ValueError('原始 CSV 含空值或非有限值，未执行转换')
    with tempfile.TemporaryDirectory(prefix='gui_motion_convert_') as directory:
        output = Path(directory) / 'converted.csv'
        if compact:
            convert_aiq11_motion_editor_csv(source, output, layout)
        else:
            convert_path(source, output, layout)
        check_motion_stream_file(output)
        return source, read_motion_csv(output, 44, strict=True), True


def simulation_file(frames, config_path, directory):
    """Export only the selected upper-body slots for the simulation file service."""
    from motion_stream_tools.layout import load_any_layout
    if not 2 <= len(frames) <= 100000:
        raise ValueError('文件须包含 2～100000 帧')
    mask = frames[0].joint_pos_mask
    if any(not frame.joint_active or frame.joint_pos_mask != mask for frame in frames):
        raise ValueError('仿真文件不支持停用关节或时变关节掩码')
    layout = load_any_layout('aiq11pro')
    hand = {29:'l_fg1_1',30:'l_fg2_0',31:'l_fg2_1',32:'l_fg3_0',33:'l_fg3_1',
            35:'r_fg1_1',36:'r_fg2_0',37:'r_fg2_1',38:'r_fg3_0',39:'r_fg3_1'}
    slots = {i: hand.get(i, name) for i, name in enumerate(layout.joint_names)
             if i >= 12 and i not in (34, 40) and mask & (1 << i)}
    if not slots:
        raise ValueError('没有可播放的上半身关节')
    directory = Path(directory)
    motion_path = directory / 'motion.csv'
    with motion_path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['time', *slots.values()])
        for frame in frames:
            writer.writerow([frame.timestamp - frames[0].timestamp,
                             *(frame.joint_position[i] for i in slots)])
    with Path(config_path).open() as stream:
        config = yaml.safe_load(stream)
    config['motions'] = [{'cmd': 255, 'name': 'GUI CSV', 'file': str(motion_path)}]
    path = directory / 'motions.yaml'
    path.write_text(yaml.safe_dump(config, sort_keys=False, allow_unicode=True))
    return path


class FilePlayback:
    def __init__(self, window):
        self.window = window
        self.phase = ''
        self.future = None
        self.frames = []
        self.publisher = None
        self.subscription = None
        self.last_tick = 0.0
        self.last_feedback = 0.0
        self.temporary = None

    @property
    def active(self):
        return bool(self.phase)

    def start(self):
        w = self.window
        w.sync_remote()
        if not w.can_send() or w.motion_client is None or not w.motion_client.service_is_ready():
            w.file_result.setText('控制器未就绪或正在被其他输入控制，不能执行文件')
            return
        if w.mode != 28:
            w.file_result.setText('请先进入 AMP WALK 并站稳，再执行文件')
            return
        try:
            path, frames, _ = load_playback_file(w.motion_file.text())
            if w.settings.get('simulation_motion_config'):
                self.temporary = tempfile.TemporaryDirectory(prefix='gui_csv_')
                config = simulation_file(frames, w.settings['simulation_motion_config'], self.temporary.name)
                self.session = 'file:' + str(config)
                w.stop_velocity(center=True)
                self.command(1)
                self.phase = 'native_starting'
                w.file_result.setText(f'准备仿真回放 {path.name}（只播放上半身）…')
                return
            from ehr_message.msg import MotionState, MotionExeStatus
            from motion_stream_tools.ros_bridge import motion_frame_to_msg, TYPE_REALTIME
            # Build and validate all messages before requesting control.
            messages = [motion_frame_to_msg(f, 44, TYPE_REALTIME, mode='mixed') for f in frames]
            topic = w.settings.get('motion_stream_topic',
                                   w.settings['motion_cmd_service'].rsplit('/', 1)[0] + '/motion_stream')
            if self.publisher is None:
                self.publisher = w.node.create_publisher(MotionState, topic, 10)
                self.subscription = w.node.create_subscription(
                    MotionExeStatus, topic.rsplit('/', 1)[0] + '/motion_exe_status',
                    self.observe, qos_profile_sensor_data)
            if w.node.count_publishers(topic) > 1:
                raise ValueError('有其他 Motion Stream 发布器，请先关闭以免混播')
            self.frames = messages
            self.times = [f.timestamp - frames[0].timestamp for f in frames]
            self.session = 'gui-file-' + uuid.uuid4().hex
            self.phase = 'discover'
            self.deadline = time.monotonic() + 3.0
            w.stop_velocity(center=True)
            w.file_result.setText(f'准备回放 {path.name}，等待文件流连接…')
        except Exception as error:
            self.finish(f'文件无法执行：{error}')

    def command(self, cmd):
        from ehr_message.srv import MotionCmd
        if self.future is not None and not self.future.done():
            self.window.motion_client.remove_pending_request(self.future)
        request = MotionCmd.Request()
        request.cmd, request.session_id = cmd, self.session
        self.future = self.window.motion_client.call_async(request)
        self.deadline = time.monotonic() + self.window.settings['service_timeout']

    def observe(self, message):
        if not self.active or self.phase in ('discover', 'stopping'):
            return
        # Older controllers leave session_id empty; use the confirmed planner source.
        if message.session_id and message.session_id != self.session:
            self.finish('文件回放被其他会话接管')
            return
        if message.header.frame_id.startswith('JOINT_REALTIME'):
            self.last_feedback = time.monotonic()
            if self.phase == 'waiting':
                self.phase = 'playing'
                self.started = self.last_tick = self.last_feedback
            elif self.phase == 'playing' and message.state in (3, 4):
                self.stop('控制器中止文件回放')
        elif self.phase == 'playing':
            self.finish('控制器已退出文件回放')

    def tick(self, healthy):
        if not self.active:
            return
        now = time.monotonic()
        w = self.window
        if self.phase == 'stopping':
            if self.future.done():
                response = self.future.result()
                self.finish(self.stop_reason if response.result else '停止请求被拒绝，请检查控制器状态')
            elif now > self.deadline:
                self.finish('停止响应超时，请检查控制器状态')
            return
        if not healthy or w.mode != 28:
            self.stop('控制状态变化，已停止文件回放')
            return
        if self.phase == 'native_starting':
            if self.future.done():
                response = self.future.result()
                if not response.result:
                    self.finish(f'仿真文件被拒绝，错误码 {response.err_code}；请检查关节限值和控制器日志')
                    return
                self.phase = 'native_playing'
                self.deadline = now + 1.0
            elif now > self.deadline:
                self.stop('仿真文件执行响应超时')
            return
        if self.phase == 'native_playing':
            text = w.detail.toPlainText()
            state = re.search(r'upper_motion_cmd=(\d+) upper_motion_phase=(\d+) upper_motion_seconds=([0-9.]+)', text)
            if state and state[1] == '255' and state[2] != '0':
                self.deadline = now + 1.0
                w.file_result.setText(f'仿真文件回放 {state[3]} 秒（只播放上半身）')
            elif now > self.deadline:
                self.finish('仿真文件回放已结束或被其他动作接管')
            return
        if w.node.count_publishers(self.publisher.topic_name) > 1:
            self.stop('检测到其他文件流发布器，已停止回放')
            return
        if self.phase == 'discover':
            if self.publisher.get_subscription_count():
                self.command(1)
                self.phase = 'starting'
            elif now > self.deadline:
                self.finish('未连接 Motion Stream 接收端，未发送执行请求')
            return
        if self.phase == 'starting':
            if self.future.done():
                response = self.future.result()
                if not response.result:
                    self.finish(f'文件执行请求被拒绝，错误码 {response.err_code}')
                    return
                self.phase = 'waiting'
                self.deadline = now + 1.0
            elif now > self.deadline:
                self.stop('文件执行响应超时')
            return
        if self.phase == 'waiting':
            if now > self.deadline:
                self.stop('未确认文件流控制器激活，已取消回放')
            return
        if now - self.last_tick > 0.15 or now - self.last_feedback > 0.5:
            self.stop('回放或反馈中断，已停止，未跳帧追赶')
            return
        self.last_tick = now
        elapsed = now - self.started
        if elapsed > self.times[-1] + 0.06:
            self.stop('文件回放结束')
            return
        index = max(0, bisect_right(self.times, elapsed) - 1)
        self.publisher.publish(self.frames[index])
        w.file_result.setText(f'文件回放 {min(elapsed, self.times[-1]):.2f}/{self.times[-1]:.2f} 秒')

    def stop(self, reason):
        if not self.active or self.phase == 'stopping':
            return
        if self.phase == 'discover':
            self.finish(reason)
            return
        self.stop_reason = reason
        self.command(2)
        self.phase = 'stopping'
        self.window.file_result.setText(reason + '；等待停止确认…')

    def finish(self, text):
        if self.future is not None and not self.future.done():
            self.window.motion_client.remove_pending_request(self.future)
        self.phase, self.frames, self.future = '', [], None
        if self.temporary is not None:
            self.temporary.cleanup()
            self.temporary = None
        self.window.file_result.setText(text)
