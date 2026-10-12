"""Chinese rollout dashboard rendered into the existing OpenCV Qt5 window."""
import math
import time

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation

FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
FONTS = {n: ImageFont.truetype(FONT, n) for n in (16, 18, 20, 22, 24, 26, 30)}
BG, PANEL, LINE = (12, 17, 23), (20, 27, 36), (45, 58, 72)
TEXT, MUTED = (229, 236, 244), (150, 166, 184)
BLUE, GREEN, YELLOW, RED = (82, 177, 239), (94, 218, 165), (246, 197, 92), (250, 112, 104)


def render_dashboard(state, frames, feedback, ages, now=None):
    now = time.time() if now is None else now
    im = Image.new('RGB', (1600, 960), BG)
    d = ImageDraw.Draw(im)

    def text(x, y, value, size=20, color=TEXT):
        d.text((x, y), str(value), font=FONTS[size], fill=color)

    def paragraph(x, y, value, width, lines=2, size=20, color=TEXT):
        row, line = '', 0
        for char in str(value or ''):
            if char == '\n' or d.textlength(row + char, font=FONTS[size]) > width:
                text(x, y + line * (size + 8), row, size, color)
                row, line = '', line + 1
                if line >= lines:
                    return
                if char == '\n':
                    continue
            row += char
        if row and line < lines:
            text(x, y + line * (size + 8), row, size, color)

    def section_title(y, title):
        text(24, y, title, 22, YELLOW)
        d.line((24, y + 34, 1256, y + 34), fill=LINE, width=1)

    def bar(x, y, width, fraction, color, indeterminate=False):
        d.rounded_rectangle((x, y, x + width, y + 12), 5, fill=(39, 50, 63))
        if indeterminate:
            left = x + int((now % 1.5) / 1.5 * (width - 150))
            d.rounded_rectangle((left, y, left + 150, y + 12), 5, fill=color)
        elif fraction > 0:
            d.rounded_rectangle((x, y, x + max(12, int(width * min(1, fraction))), y + 12), 5, fill=color)

    def chinese_plan(value):
        value = str(value or '等待模型规划')
        normalized = value.strip().lower().rstrip('.。')
        mappings = (
            ('right arm failed to settle; no subsequent motion allowed', '右臂未达到稳定状态，后续动作已停止'),
            ('right arm movement is not allowed', '当前不允许右臂移动'),
            ('right arm action not allowed', '当前不允许执行该右臂动作'),
            ('right arm tracking error to emitted command exceeds', '右臂跟踪误差超过限制'),
            ('IK residual exceeds small-action tolerance', '逆运动学残差超过小动作容差'),
            ('trial lift needs fresh object-follows-hand evidence', '试抬后需要新图像确认蛋糕随夹爪移动'),
            ('first motion after close must be', '夹爪闭合后的第一步必须是纯向上试抬'),
            ('observation expired', '观测已过期，需要重新获取相机画面'),
            ('operator STOP', '操作员请求停止'),
            ('HOLD', '保持当前位置'),
            ('action ', '当前动作'),
        )
        for source, target in mappings:
            if source in value:
                suffix = value.split(source, 1)[1].lstrip(':：;； ')
                if source == 'action ' and 'not allowed in' in suffix:
                    action, stage = suffix.split('not allowed in', 1)
                    return f'当前阶段不允许执行 {action.strip()} 动作（阶段：{stage.strip()}）'
                return target + (f'：{suffix}' if suffix and source not in ('HOLD',) else '')
        if normalized.startswith('right arm ') and normalized.endswith(' allowed'):
            return '当前允许执行右臂动作'
        return value

    iteration = int(state.get('iteration', 0) or 0)
    text(24, 14, '任务：将蛋糕放入蓝色盘子里', 30, BLUE)
    text(1000, 18, f'|    Step：{iteration:02d}', 26, BLUE)
    d.line((20, 58, 1260, 58), fill=LINE, width=1)

    for label, index, key, x in (
        ('俯视相机', 1, 'head_right', 24),
        ('腕部相机', 3, 'wrist_right', 646),
    ):
        age = ages.get(key, 0)
        camera_ok = frames is not None and age <= 2
        text(x, 68, label, 18, TEXT)
        text(x + 485, 68, '实时' if camera_ok else '异常', 16, TEXT if camera_ok else RED)
        if frames is not None and len(frames) > index and frames[index] is not None:
            rgb = cv2.cvtColor(cv2.resize(frames[index], (596, 278)), cv2.COLOR_BGR2RGB)
            im.paste(Image.fromarray(rgb), (x, 96))
            d.rectangle((x, 96, x + 596, 374), outline=LINE, width=1)
        else:
            d.rectangle((x, 96, x + 596, 374), fill=(10, 14, 19), outline=LINE)

    section_title(392, '观测')
    paragraph(24, 432, state.get('observation_text', '正在整合俯视与腕部相机观测'), 1228, 2, 20)

    section_title(496, '当前规划')
    stage_labels = {
        'approach': '接近目标', 'capture': '精确对准', 'close': '执行抓取',
        'trial_lift': '试抬验证', 'raise': '抬升净空', 'transfer': '移动到盘',
        'lower': '下降放置', 'release': '释放蛋糕', 'verify': '判断任务结束',
    }
    stage = state.get('stage', 'approach')
    paragraph(24, 536, chinese_plan(state.get('action_prompt', state.get('reason', '等待模型规划'))), 1228, 2, 20)

    delta = state.get('planned_delta_pose')
    if delta:
        xyz = np.asarray(delta.get('translation_m', [0, 0, 0]), dtype=float) * 1000
        rotvec = np.asarray(delta.get('rotvec_rad', [0, 0, 0]), dtype=float)
        try:
            euler = Rotation.from_rotvec(rotvec).as_euler('xyz', degrees=True)
        except ValueError:
            euler = np.zeros(3)
        text(24, 606, f"Delta XYZ：X {xyz[0]:+.1f}  Y {xyz[1]:+.1f}  Z {xyz[2]:+.1f} mm", 18, BLUE)
        text(620, 606, f"Delta Euler：R {euler[0]:+.2f}°  P {euler[1]:+.2f}°  Y {euler[2]:+.2f}°", 18, BLUE)
    else:
        text(24, 606, 'Delta：等待本 Step 规划输出', 18, MUTED)

    section_title(650, '控制流')
    commanded_grip = float(state.get('right_gripper_ratio', 0) or 0)
    measured_grip = feedback.get('hand_ratio', float('nan'))
    grip_action = '闭合' if commanded_grip >= .5 else '打开'
    grip_state = ('闭合' if measured_grip >= .5 else '打开') if math.isfinite(measured_grip) else '未知'
    delta_status = state.get('delta_status', '等待动作')
    hz = float(feedback.get('control_hz', 0) or 0)
    error_words = ('异常', '失败', '超时', '停止', '未确认', '拒绝', '错误')
    body_error = any(word in str(delta_status) for word in error_words)
    text(24, 690, '本体状态检测：', 18, TEXT)
    text(188, 690, f'夹爪 Action：{grip_action}  |  State：{grip_state}', 18, MUTED)
    text(530, 690, f'末端 Action：{stage_labels.get(stage, stage)}  |  State：{delta_status}', 18,
         RED if body_error else MUTED)

    camera_age = float(feedback.get('camera_age', 999))
    dds_age = float(feedback.get('dds_age', 999))
    head_age, wrist_age = float(ages.get('head_right', 0)), float(ages.get('wrist_right', 0))
    camera_ok = camera_age < .5 and head_age <= 2 and wrist_age <= 2
    control_hz = f'{hz:.1f} Hz' if dds_age < .5 and hz > 0 else '等待实时反馈'
    text(1015, 690, f'控制频率：{control_hz}', 18, MUTED if dds_age < .5 else RED)
    text(24, 730, '相机状态检测：', 18, TEXT)
    text(188, 730, '双路实时 / 图像流正常' if camera_ok else '相机流异常或画面冻结', 18,
         MUTED if camera_ok else RED)
    text(530, 730, f'包龄 {camera_age:.2f}s', 18, MUTED)
    text(700, 730, f'DDS {dds_age:.2f}s', 18, MUTED if dds_age < .5 else RED)

    planning = state.get('planning_phase', 'observing')
    planning_titles = {'observing': '获取观测', 'inference': '模型推理中', 'checking': '规划校验',
                       'ready': '规划完成', 'executing': '规划完成', 'done': '规划完成', 'hold': '等待下一步'}
    planning_fraction = {'observing': .15, 'checking': .78, 'ready': 1, 'executing': 1,
                         'done': 1, 'hold': 1}.get(planning, 0)
    if planning == 'executing':
        execution_fraction = float(state.get('execution_progress', .55) or 0)
    elif planning == 'done' or str(delta_status).startswith('已下发'):
        execution_fraction = 1
    else:
        execution_fraction = 0
    inference_s = float(state.get('inference_s', 0) or 0)
    if planning == 'inference':
        inference_s = max(0, now - float(state.get('planning_started_at', now)))
    execution_s = float(state.get('execution_s', 0) or 0)
    text(24, 770, f"本次规划：{planning_titles.get(planning, planning)}  |  推理时间 {inference_s:.2f}s", 18)
    bar(24, 802, 1226, planning_fraction, BLUE, planning == 'inference')
    text(24, 826, f"动作执行：{delta_status}  |  执行时间 {execution_s:.2f}s", 18)
    bar(24, 858, 1226, execution_fraction, GREEN)
    total_elapsed_s = float(feedback.get('ui_elapsed_s', 0) or 0)
    total_minutes, centiseconds = divmod(int(round(max(0, total_elapsed_s)*100)), 6000)
    total_seconds = centiseconds / 100
    text(24, 888, f"推理总时长：{int(total_minutes)}分 {total_seconds:05.2f}秒", 24, TEXT)

    d.rectangle((1280, 0, 1599, 959), fill=(21, 28, 38))
    text(1304, 24, '任务阶段', 26)
    text(1304, 62, '蛋糕 → 蓝色盘子', 18, MUTED)
    stages = [
        ('approach', '01  接近', '进入可抓取工作区'),
        ('capture', '02  对准', '两指对齐蛋糕抓取带'),
        ('close', '03  抓取', '闭合夹爪稳定夹持'),
        ('raise', '04  抬升', '建立桌面与盘沿净空'),
        ('transfer', '05  到盘', '移动至蓝盘中心上方'),
        ('lower', '06  下降', '下降至盘内支撑高度'),
        ('release', '07  释放', '确认支撑后打开夹爪'),
        ('verify', '08  结束判断', '盘内稳定且夹爪撤离'),
    ]
    active_stage = {'trial_lift': 'raise'}.get(stage, stage)
    for i, (key, title, subtitle) in enumerate(stages):
        y = 122 + i * 94
        active = key == active_stage
        if i < len(stages) - 1:
            d.line((1318, y + 28, 1318, y + 91), fill=LINE, width=2)
        d.ellipse((1308, y + 10, 1328, y + 30), fill=GREEN if active else (61, 77, 95))
        text(1344, y, title, 20, GREEN if active else TEXT)
        paragraph(1344, y + 34, subtitle, 220, 2, 16, MUTED)

    return cv2.cvtColor(np.asarray(im), cv2.COLOR_RGB2BGR)
