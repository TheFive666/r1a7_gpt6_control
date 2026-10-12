#!/usr/bin/env python3
"""Astra zero-shot right Dex1 visual servo, phase 1 ONLY (never close/lift).

--probe reads the existing PC2 stream and DDS without opening any publishers.
Actual movement additionally requires --motion-mode --enable-dex1 and a valid
command handover, an API credential and known fixed waist position.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
import signal
from pathlib import Path
import sys
import threading
import time

REPO_ROOT = Path(__file__).resolve().parents[2]
ROOT = REPO_ROOT
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'third_party' / 'unitree_sdk2_python'))

import numpy as np

from agent.astra_policy import AstraClient, Halt, Phase, selected_views, validate_decision
from teleop.image_client import FourCameraSubscriber


class ReadOnlyDDS:
    """Observe handover commands and feedback; never publishes or changes mode."""
    def __init__(self, interface):
        from robot.r1_a7_real import REAL_JOINTS
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
        self.real_joints = REAL_JOINTS
        self.lock = threading.Lock()
        self.messages = {}
        self.times = {}
        self.subscribers = []
        ChannelFactoryInitialize(0, interface)
        specs = [('lowstate', 'rt/lowstate', LowState_), ('lowcmd', 'rt/lowcmd', LowCmd_),
                 ('left_cmd', 'rt/dex1/left/cmd', MotorCmds_),
                 ('right_cmd', 'rt/dex1/right/cmd', MotorCmds_),
                 ('left_state', 'rt/dex1/left/state', MotorStates_),
                 ('right_state', 'rt/dex1/right/state', MotorStates_)]
        for name, topic, kind in specs:
            sub = ChannelSubscriber(topic, kind)
            sub.Init(lambda msg, key=name: self.receive(key, msg), 1)
            self.subscribers.append(sub)

    def receive(self, name, msg):
        with self.lock:
            self.messages[name] = msg
            self.times[name] = time.monotonic()

    def require_fresh(self):
        with self.lock:
            for key in ('lowstate', 'left_state', 'right_state'):
                if time.monotonic() - self.times.get(key, -float('inf')) > 0.5:
                    raise Halt(f"missing/stale DDS {key}")
            state = self.messages['lowstate']
            q = [float(state.motor_state[i].q) for _, i, _, _ in self.real_joints]
            hands = [float(self.messages[k].states[0].q) for k in ('left_state', 'right_state')]
        if not np.all(np.isfinite(q + hands)):
            raise Halt('invalid DDS position feedback')

    def require_quiet_commands(self):
        with self.lock:
            active = [k for k in ('lowcmd', 'left_cmd', 'right_cmd') if time.monotonic() - self.times.get(k, -float('inf')) < 0.75]
        if active:
            raise Halt('another command publisher is active: ' + ', '.join(active) + '; stop it after saving the handover')

    def arm_head_positions(self):
        with self.lock:
            state = self.messages['lowstate']
            return np.array([float(state.motor_state[i].q) for _, i, _, _ in self.real_joints])

    def snapshot(self, waist, hold_measured=False):
        self.require_fresh()
        if waist is None or not np.isfinite(waist):
            raise Halt('known fixed --waist-rad is required; waist state index is not defined by the existing bridge')
        with self.lock:
            cmd = self.messages.get('lowcmd')
            if cmd is None and not hold_measured:
                raise Halt('no current rt/lowcmd observed; provide a saved command --handover instead of guessing left commands')
            if hold_measured:
                state = self.messages['lowstate']
                if any(abs(float(state.motor_state[i].dq)) > 0.02 for _, i, _, _ in self.real_joints):
                    raise Halt('cannot capture measured hold while joints are moving')
                joints = {name: float(state.motor_state[i].q) for name, i, _, _ in self.real_joints}
            else:
                motors = [cmd.motor_cmd[i] for _, i, _, _ in self.real_joints]
                if any(m.mode != 1 for m in motors):
                    raise Halt('handover arm/head motors are not position-controlled')
                joints = {name: float(cmd.motor_cmd[i].q) for name, i, _, _ in self.real_joints}
            joints['waist_yaw_joint'] = float(waist)
            left = self.messages.get('left_cmd')
            left_position = float(left.cmds[0].q) if left else float(self.messages['left_state'].states[0].q)
        if not 0 <= left_position <= 5:
            raise Halt('left Dex1 position outside supported ratio range')
        return {'version': 1, 'created_at_ns': time.time_ns(),
                'source': 'explicit_measured_hold' if hold_measured else 'observed_commands', 'joint_commands': joints,
                'left_closed_ratio': 1.0 - left_position / 5.0}

    def close(self):
        for sub in self.subscribers:
            sub.Close()


def fresh_frames(sub, last_id, after_ns, timeout=3.0, watchdog=lambda: None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        watchdog()
        frames, frame_id, received = sub.get_if_new(last_id)
        if frames is not None and frame_id > last_id and received > after_ns and time.time_ns() - received < 500_000_000:
            if any(f is None or f.size == 0 for f in (frames[0], frames[1], frames[3])):
                raise Halt('empty image')
            if np.array_equal(frames[0], frames[1]):
                raise Halt('head stream is duplicated monocular data, not Head Stereo')
            return frames, frame_id, received
        time.sleep(0.02)
    raise Halt('no fresh post-action PC2 image packet')


class Journal:
    def __init__(self, path):
        self.path = Path(path)
        self.path.mkdir(parents=True, exist_ok=False)

    def write(self, event, **values):
        with (self.path / 'events.jsonl').open('a', encoding='utf-8') as f:
            f.write(json.dumps({'time_ns': time.time_ns(), 'event': event, **values}, ensure_ascii=False, allow_nan=False) + '\n')

    def images(self, iteration, views):
        import cv2
        for name, frame in views:
            if not cv2.imwrite(str(self.path / f'{iteration:04d}_{name}.jpg'), frame):
                raise Halt('could not write observation log')


def run_loop(args, controller, sub, probe, client, journal):
    phase = Phase()
    last_id = 0
    after_ns = time.time_ns()
    previous = None
    view_state = {'mode': 'rollout', 'phase': 'HEAD OBSERVE', 'start': 0., 'target': 0., 'right_gripper_ratio': 0.0}
    def show(**values):
        if args.view_dir is None:
            return
        view_state.update(values)
        path = args.view_dir / 'status.tmp'
        path.write_text(json.dumps(view_state), encoding='utf-8')
        path.replace(args.view_dir / 'status.json')
    executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='astra-request')

    def watchdog():
        probe.require_fresh()
        if args.view_dir is not None:
            if (args.view_dir / 'stop_requested').exists():
                raise Halt('operator STOP requested in visualization')
            heartbeat = args.view_dir / 'view_alive'
            if not heartbeat.exists() or time.time() - heartbeat.stat().st_mtime > 1:
                raise Halt('visualization stopped or unresponsive')
        with sub.lock:
            age = time.time_ns() - sub.received_at_ns
        if age > 500_000_000:
            raise Halt('PC2 stream stopped')
        if controller.motion_mode:
            real = controller.arm_ctrl.real
            if real.writer_thread is None or not real.writer_thread.is_alive():
                raise Halt('DDS arm writer stopped')
            if real.last_lowstate_time is None or time.monotonic() - real.last_lowstate_time > 0.5:
                raise Halt('arm controller lowstate is stale')
            actual = probe.arm_head_positions()
            if np.max(np.abs(actual[:7] - controller.initial_command[:7])) > 0.08:
                raise Halt('left arm moved away from held command')
            if np.max(np.abs(actual[14:] - controller.initial_command[14:])) > 0.08:
                raise Halt('head moved away from held command')
            hand_thread = controller.hand_ctrl.gripper_ctrl.gripper_control_thread
            if not hand_thread.is_alive():
                raise Halt('Dex1 writer stopped')

    try:
        for iteration in range(args.max_steps):
            frames, last_id, received = fresh_frames(sub, last_id, after_ns, watchdog=watchdog)
            controller.refresh()
            p, quat = controller.pose()
            measured_p, measured_quat = controller.measured_pose()
            show(phase=phase.stage.upper() + ' / ASTRA OBSERVE',
                 reason='Waiting for one Astra decision; arm holds.', iteration=iteration,
                 ee_position_mm=(measured_p * 1000).tolist())
            views = selected_views(frames, phase.stage)
            journal.images(iteration, views)
            observation = {'target': args.target, 'phase': phase.stage, 'frame_id': last_id,
                           'ee_position_base_m': p.tolist(), 'ee_quaternion_xyzw': quat.tolist(),
                           'ee_pose_source': 'persistent commanded FK, matching Pico teleop',
                           'measured_ee_position_base_m': measured_p.tolist(),
                           'measured_ee_quaternion_xyzw': measured_quat.tolist(),
                           'urdf_tool_geometry': controller.tool_geometry,
                           'workspace_base_m': controller.workspace.tolist(),
                           'previous_executed_action': previous, 'gripper_closed_ratio': 0.0,
                           'instructions': 'Choose exactly one small action or a zero-motion phase decision.'}
            journal.write('observation', iteration=iteration, observation=observation, received_at_ns=received)
            request = executor.submit(client.decide, observation, views)
            deadline = time.monotonic() + args.max_decision_age
            while not request.done():
                watchdog()
                if time.monotonic() >= deadline:
                    raise Halt('Astra decision deadline exceeded; late output discarded')
                time.sleep(0.02)
            raw = request.result()
            if time.time_ns() - received > args.max_decision_age * 1e9:
                raise Halt('Astra output belongs to an expired observation')
            watchdog()
            # The arm is stationary during inference. Discard a proposal if
            # its scene changed substantially while the service was thinking.
            latest, _, _ = sub.get()
            for (_, old), (_, new) in zip(views, selected_views(latest, phase.stage)):
                if old.shape != new.shape or np.mean(np.abs(old.astype(float) - new.astype(float))) / 255 > 0.06:
                    raise Halt('scene changed during Astra inference; proposal discarded')
            journal.write('decision', iteration=iteration, decision=raw)
            decision = validate_decision(raw, phase.stage)
            show(phase=phase.stage.upper() + ' / ' + decision.action.upper(), reason=decision.reason)
            print(f'[{iteration:03d} {phase.stage}] {decision.action}: {decision.reason}', flush=True)
            if decision.action == 'move':
                q = controller.solve_step(decision)
                watchdog()
                if time.time_ns() - received > args.max_decision_age * 1e9:
                    raise Halt('observation expired during IK')
                if not args.motion_mode:
                    journal.write('proposal_only', right_q=q[controller.right_arm_q_indices].tolist())
                    return 'OBSERVE_ONLY: validated one proposal, no commands published'
                controller.execute_one(q, watchdog)
                previous = {'translation_base_m': list(decision.translation), 'rotvec_base_rad': list(decision.rotation),
                            'source_frame_id': last_id,
                            'commanded_right_q': controller.q[controller.right_arm_q_indices].tolist(),
                            'measured_right_q': controller.measured_q[controller.right_arm_q_indices].tolist()}
                journal.write('action_settled', iteration=iteration, action=previous)
                measured_p, _ = controller.measured_pose()
                show(phase='ACTION SETTLED / NEW IMAGE', ee_position_mm=(measured_p * 1000).tolist())
            done = phase.accept(decision)
            # Even phase confirmations must use a NEW packet after the previous
            # decision/action completed. No cached-image retries are possible.
            after_ns = time.time_ns()
            if done:
                journal.write('pregrasp_stable', confirmations=3)
                return 'PREGRASP_STABLE: right hand open; no close or lift'
        raise Halt('visual action budget exhausted')
    finally:
        executor.shutdown(wait=False, cancel_futures=True)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--target', default='桌子上的胡萝卜；对准适合夹持的中部，避开细尖端，第一阶段停在无接触的 pre-grasp')
    p.add_argument('--motion-mode', action='store_true')
    p.add_argument('--enable-dex1', action='store_true')
    p.add_argument('--net', default='enp5s0')
    p.add_argument('--image-address', default='192.168.123.164')
    p.add_argument('--image-port', type=int, default=5555)
    p.add_argument('--probe', action='store_true', help='read-only camera/DDS check; no API or motion')
    p.add_argument('--save-handover', type=Path, help='save currently observed command snapshot during --probe')
    p.add_argument('--handover', type=Path, help='command snapshot saved before stopping the previous controller')
    p.add_argument('--waist-rad', type=float, help='known fixed waist position, needed when capturing handover')
    p.add_argument('--hold-measured', action='store_true', help='explicitly adopt stationary measured arm/head positions as hold commands when no prior command exists')
    p.add_argument('--model', default='gpt-6-astra', help='exact model ID exposed by your Astra API; never substituted')
    p.add_argument('--base-url', default='https://ys3.chunxueai.com/v1')
    p.add_argument('--api-key-env', default='UNITREE_OPENAI_API_KEY')
    p.add_argument('--api-proxy', default='http://127.0.0.1:7897', help='HTTP proxy for Astra requests only; DDS/ZMQ remain direct')
    p.add_argument('--view-dir', type=Path, help='status directory of the running visualization; STOP and heartbeat are monitored')
    p.add_argument('--takeover-pid', type=int, help='replace a known holding test controller after all prechecks')
    p.add_argument('--max-decision-age', type=float, default=15.0)
    p.add_argument('--max-steps', type=int, default=60)
    p.add_argument('--reset-blend-frames', type=int, default=100)
    p.add_argument('--workspace', type=float, nargs=6, default=[-0.2, 0.8, -0.85, 0.05, -0.45, 0.7],
                   metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX', 'ZMIN', 'ZMAX'), help='EE bounds in pelvis/base metres; not a table model')
    p.add_argument('--log-dir', type=Path, default=ROOT / 'logs' / ('astra_' + time.strftime('%Y%m%d_%H%M%S')))
    return p


def hold_until_home(controller, probe, journal):
    """A latched hold never resumes autonomous actions. Exit only via home blend."""
    controller.freeze()
    print('保持当前位置。输入 home 或按 Ctrl+C：检查后仅右臂插值回 home 并退出。', flush=True)
    while True:
        try:
            text = input('hold> ').strip().lower()
            if text != 'home':
                continue
        except EOFError:
            # Detached session: keep DDS alive at the frozen command.
            try:
                time.sleep(0.2)
                continue
            except KeyboardInterrupt:
                pass
        except KeyboardInterrupt:
            pass
        try:
            controller.return_home(probe.require_fresh)
            journal.write('home_complete')
            return
        except (Exception, KeyboardInterrupt) as exc:
            controller.freeze()
            journal.write('home_blocked', reason=str(exc))
            print(f'Home 被阻止，继续保持：{exc}', flush=True)


def main():
    args = parser().parse_args()
    if args.motion_mode and not args.enable_dex1:
        raise SystemExit('--motion-mode requires --enable-dex1 (phase 1 remains open)')
    if args.max_steps < 1 or args.reset_blend_frames < 100 or not 0 < args.max_decision_age <= 30:
        raise SystemExit('require max-steps >=1, reset-blend-frames >=100, decision age in (0,30]')
    bounds = np.array(args.workspace).reshape(3, 2)
    if not np.all(np.isfinite(bounds)) or np.any(bounds[:, 0] >= bounds[:, 1]):
        raise SystemExit('invalid workspace bounds')
    if args.save_handover and not args.probe:
        raise SystemExit('--save-handover is only valid with --probe')
    sub = FourCameraSubscriber(args.image_address, args.image_port)
    probe = controller = None
    journal = Journal(args.log_dir)
    try:
        sub.start()
        frames, frame_id, received = fresh_frames(sub, 0, time.time_ns(), timeout=8)
        journal.images(0, selected_views(frames, 'head') + selected_views(frames, 'wrist'))
        print('PC2: head_left, head_right, wrist_right ready; wrist_left unused', flush=True)
        probe = ReadOnlyDDS(args.net)
        time.sleep(1.5)
        probe.require_fresh()
        with probe.lock:
            status = {k: round(time.monotonic() - v, 3) for k, v in probe.times.items()}
        journal.write('probe', cameras=['head_left', 'head_right', 'wrist_right'], dds_age_s=status)
        print('DDS observed:', status, flush=True)
        if args.probe:
            if args.save_handover:
                snapshot = probe.snapshot(args.waist_rad, args.hold_measured)
                args.save_handover.parent.mkdir(parents=True, exist_ok=True)
                with args.save_handover.open('x', encoding='utf-8') as f:
                    json.dump(snapshot, f, ensure_ascii=False, indent=2, allow_nan=False)
                print(f'Saved handover: {args.save_handover}')
            return 0
        # Credentials checked BEFORE opening any arm/hand publisher.
        client = AstraClient(args.model, args.base_url, args.api_key_env, proxy=args.api_proxy or None)
        if args.handover:
            snapshot = json.loads(args.handover.read_text(encoding='utf-8'))
            if snapshot.get('version') != 1:
                raise Halt('unsupported handover version')
        else:
            snapshot = probe.snapshot(args.waist_rad, args.hold_measured)
        from robot.config import R1A7Config
        from agent.astra_robot import RightOnlyController
        controller = RightOnlyController(R1A7Config(reset_blend_frames=args.reset_blend_frames), snapshot, args.workspace)
        # Building URDF meshes/Placo can hold the GIL for seconds; allow DDS
        # callbacks to receive NEW samples before any actuation/readiness test.
        ready_deadline = time.monotonic() + 2.0
        while True:
            time.sleep(.02)
            try:
                probe.require_fresh()
                break
            except Halt:
                if time.monotonic() >= ready_deadline:
                    raise
        if args.motion_mode:
            if args.takeover_pid is not None:
                command = Path(f'/proc/{args.takeover_pid}/cmdline').read_bytes()
                if b'scripts/test_right_ee_delta.py' not in command:
                    raise Halt('takeover PID is not the known EE-test controller')
                probe.require_fresh()
                # The new initial targets must exactly preserve the held DDS
                # commands before transferring ownership of the writer.
                with probe.lock:
                    held = probe.messages.get('lowcmd')
                    held_q = np.array([float(held.motor_cmd[i].q) for _, i, _, _ in probe.real_joints]) if held else None
                if held_q is None or not np.allclose(held_q, controller.initial_command, rtol=0, atol=1e-7):
                    raise Halt('held commands changed since handover capture')
                os.kill(args.takeover_pid, signal.SIGTERM)
                deadline = time.monotonic() + 2
                while Path(f'/proc/{args.takeover_pid}').exists():
                    if time.monotonic() > deadline:
                        raise Halt('previous test writer did not stop')
                    time.sleep(.01)
                # Wait for prior command samples to age out of the publisher
                # conflict check. The robot remains in its debugging mode.
                time.sleep(.8)
            controller.attach_hardware(args.net, probe)
            # Right gripper opening is the only startup action. Observe only
            # after fresh state confirms open, rather than trusting a command.
            deadline = time.monotonic() + 4.0
            while True:
                probe.require_fresh()
                with probe.lock:
                    opened = abs(float(probe.messages['right_state'].states[0].q) - 5.0) < 0.15
                if opened:
                    break
                if time.monotonic() > deadline:
                    raise Halt('right Dex1 did not reach open position')
                time.sleep(0.02)
        result = run_loop(args, controller, sub, probe, client, journal)
        print(result, flush=True)
        journal.write('result', result=result)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        if controller is not None:
            controller.freeze()
        reason = str(exc) or 'operator interruption'
        if args.view_dir is not None:
            status_path = args.view_dir / 'status.json'
            try:
                view_state = json.loads(status_path.read_text())
            except (OSError, ValueError):
                view_state = {}
            view_state.update(mode='rollout', phase='HOLD / ROLLOUT STOP', reason=reason)
            status_path.write_text(json.dumps(view_state))
        journal.write('halt', reason=reason)
        print('HALT:', reason, flush=True)
        return 2
    finally:
        if controller is not None:
            if controller.ready:
                hold_until_home(controller, probe, journal)
            controller._on_stop()
        if probe is not None:
            probe.close()
        sub.close()
        print('Log:', journal.path, flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
