#!/usr/bin/env python3
"""Run under systemd --user. One persistent IK/DDS owner, no automatic recipe."""
import argparse
import os
import signal
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
ROOT = REPO_ROOT
sys.path.insert(0, str(ROOT))
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from robot.config import R1A7Config
from agent.scripts.astra_grasp_rollout import ReadOnlyDDS
from agent.scripts.fixed_cake_rollout import snapshot, show
from agent.astra_policy import Halt
from agent.astra_robot import RightOnlyController
from teleop.image_client import FourCameraSubscriber
from agent.resident_control import ResidentInbox, atomic_json


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--net', default='enp5s0')
    p.add_argument('--view-dir', type=Path, required=True)
    p.add_argument('--held-pid', type=int)
    p.add_argument('--motion-mode', action='store_true')
    p.add_argument('--operator-debug-confirmed', action='store_true')
    a = p.parse_args()
    if not a.motion_mode or not a.operator_debug_confirmed:
        p.error('real service requires --motion-mode and operator-established debugging mode')
    a.view_dir.mkdir(parents=True, exist_ok=True)
    probe = ReadOnlyDDS(a.net)
    camera = FourCameraSubscriber('192.168.123.164', 5555)
    camera.start()
    time.sleep(1)
    probe.require_fresh()
    initial = snapshot(probe)
    c = RightOnlyController(R1A7Config(), initial, [-1, 1, -1, 1, -1, 1], position_checks=False)
    state = dict(iteration=0, stage='approach', planning_phase='hold', right_gripper_ratio=c.right_ratio)
    pending_signal = []
    # Signal handlers only latch; hardware work remains outside the handler.
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, lambda signum, frame: pending_signal.append(signum))
    c.attach_hardware(a.net, probe, held_pid=a.held_pid)
    with probe.lock:
        baseline = (probe.messages['lowstate'].mode_machine, probe.messages['lowstate'].mode_pr)

    def watchdog():
        probe.require_fresh()
        if pending_signal or (a.view_dir / 'stop_requested').exists():
            raise Halt('operator/signal HOLD')
        alive = a.view_dir / 'view_alive'
        if not alive.exists() or time.time() - alive.stat().st_mtime > 3:
            raise Halt('viewer stale')
        with probe.lock:
            msg = probe.messages['lowstate']
            if (msg.mode_machine, msg.mode_pr) != baseline:
                raise Halt('robot mode fields changed')
        real = c.arm_ctrl.real
        if not real.writer_thread.is_alive() or time.monotonic() - (real.last_publish_time or 0) > .2:
            raise Halt('DDS publisher unhealthy')

    def observe():
        after = time.time_ns()
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            frames, fid, stamp = camera.get()
            if frames is not None and stamp > after:
                paths = {}
                for name, idx in [('top', 1), ('wrist', 3)]:
                    path = a.view_dir / f'resident_{state["iteration"]:03d}_{name}.jpg'
                    cv2.imwrite(str(path), frames[idx])
                    paths[name] = str(path)
                return dict(images=paths, frame_id=int(fid))
            time.sleep(.02)
        raise Halt('no fresh image')

    def execute(request):
        action = request['action']
        if action == 'observe':
            return observe()
        if action == 'resume':
            if not request.get('operator_debug_confirmed'):
                raise Halt('operator debug confirmation required')
            pending_signal.clear()
            (a.view_dir / 'stop_requested').unlink(missing_ok=True)
            watchdog()
            return dict(resumed=True)
        watchdog()
        state['iteration'] += 1
        dp = np.asarray(request.get('dpos', [0, 0, 0]), dtype=float)
        dr = np.asarray(request.get('drot_deg', [0, 0, 0]), dtype=float)
        if dp.shape != (3,) or dr.shape != (3,) or not np.isfinite([dp, dr]).all():
            raise Halt('invalid delta')
        if np.linalg.norm(dp) > .030001 or np.linalg.norm(dr) > 5.0001:
            raise Halt('one action limited to 3 cm / 5 degrees')
        if action != 'move' and (np.any(dp) or np.any(dr)):
            raise Halt('non-motion action must have zero delta')
        rot = Rotation.from_euler('xyz', dr, degrees=True)
        show(a.view_dir, state, planning_phase='executing', execution_progress=0,
             stage=request.get('stage', 'approach'), action_prompt=request.get('text', action),
             observation_text=request.get('observation', ''), delta_status='正在执行',
             planned_delta_pose=dict(translation_m=dp.tolist(), rotvec_rad=rot.as_rotvec().tolist()))
        started = time.monotonic()
        def progress(value):
            show(a.view_dir, state, execution_progress=value, execution_s=time.monotonic()-started)
        if action == 'move':
            pos, quat = c.pose()
            solved = c.placo_solver.solve(c.q, {'right': (pos + dp, (rot * Rotation.from_quat(quat)).as_quat())})
            if solved is None or not np.isfinite(solved).all():
                raise Halt('IK failed')
            watchdog()
            target = c.q.copy()
            target[c.right_arm_q_indices] = solved[c.right_arm_q_indices]
            c.execute_one(target, watchdog, timeout=5, position_tolerance_m=.010, progress_callback=progress)
        elif action in ('open', 'close'):
            c.right_ratio = float(action == 'close')
            c.hand_ctrl.set_gripper_ratios(c.left_ratio, c.right_ratio)
            for tick in range(20):
                watchdog()
                progress((tick+1)/20)
                time.sleep(.05)
        else:
            raise Halt('unsupported action')
        images = observe()
        atomic_json(a.view_dir / 'commanded_state.json', dict(q=c.q.tolist(), right_ratio=c.right_ratio))
        show(a.view_dir, state, planning_phase='done', execution_progress=1,
             right_gripper_ratio=c.right_ratio, delta_status='已下发并执行')
        return dict(**images, feedback=getattr(c, 'last_execution', {}))

    def freeze():
        c.freeze()
        show(a.view_dir, state, planning_phase='hold', delta_status='保持当前指令', right_gripper_ratio=c.right_ratio)

    inbox = ResidentInbox(a.view_dir / 'inbox', execute, freeze)
    freeze()
    while True:
        try:
            if pending_signal:
                inbox.halt()
                pending_signal.clear()
            if (a.view_dir / 'stop_requested').exists() and not inbox.halted:
                inbox.halt()
            inbox.poll()
            atomic_json(a.view_dir / 'service_health.json', dict(pid=os.getpid(), time=time.time(),
                holding=inbox.halted, writer_alive=c.arm_ctrl.real.writer_thread.is_alive(),
                last_publish_age=time.monotonic()-(c.arm_ctrl.real.last_publish_time or 0)))
            time.sleep(.05)
        except BaseException as exc:
            inbox.halt()
            atomic_json(a.view_dir / 'service_fault.json', dict(error=str(exc), time=time.time()))
            time.sleep(.1)


if __name__ == '__main__':
    main()
