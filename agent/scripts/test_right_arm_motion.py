#!/usr/bin/env python3
"""One small right-wrist pitch test, followed by a 100-frame return.

Requires the existing PC2 stream. Default is read-only; --motion-mode enables
one -0.02 rad wrist pitch target. No hand controller or motion-mode switch.
"""
from pathlib import Path
import argparse
import json
import subprocess
import sys
import time

REPO_ROOT = Path(__file__).resolve().parents[2]
ROOT = REPO_ROOT
sys.path.insert(0, str(ROOT))

import numpy as np

from agent.scripts.astra_grasp_rollout import ReadOnlyDDS, Journal, fresh_frames
from agent.astra_policy import Halt, selected_views
from teleop.image_client import FourCameraSubscriber


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--net', default='enp5s0')
    parser.add_argument('--motion-mode', action='store_true')
    args = parser.parse_args()
    journal = Journal(ROOT / 'logs' / ('right_arm_test_' + time.strftime('%Y%m%d_%H%M%S')))
    sub = FourCameraSubscriber('192.168.123.164', 5555)
    probe = ctrl = None
    success = False
    status = {'phase': 'PRECHECK'}
    def show(phase=None, **values):
        if phase is not None:
            status['phase'] = phase
        status.update(values)
        temporary = journal.path / 'status.tmp'
        temporary.write_text(json.dumps(status), encoding='utf-8')
        temporary.replace(journal.path / 'status.json')

    show()
    view_log = (journal.path / 'view.log').open('w')
    viewer = subprocess.Popen([sys.executable, str(ROOT / 'agent/scripts/right_arm_test_view.py'), str(journal.path)],
                              stdout=view_log, stderr=subprocess.STDOUT, start_new_session=True)
    def check_view():
        if (journal.path / 'stop_requested').exists():
            raise Halt('operator requested STOP in visualization')
        heartbeat = journal.path / 'view_alive'
        if viewer.poll() is not None or not heartbeat.exists() or time.time() - heartbeat.stat().st_mtime > 1:
            raise Halt('visualization not running or unresponsive')

    try:
        sub.start()
        # No arm publisher can exist before this check succeeds.
        frames, fid, _ = fresh_frames(sub, 0, time.time_ns(), timeout=8)
        journal.images(0, selected_views(frames, 'head') + selected_views(frames, 'wrist'))
        probe = ReadOnlyDDS(args.net)
        time.sleep(1)
        probe.require_fresh()
        probe.require_quiet_commands()
        samples = []
        for _ in range(20):
            probe.require_fresh()
            samples.append(probe.arm_head_positions())
            time.sleep(.02)
        if np.max(np.ptp(np.asarray(samples), axis=0)) > .01:
            raise Halt('arm/head not stationary enough for a small motion test')
        initial = samples[-1]
        show('READY', start=float(initial[12]), measured=float(initial[12]), target=float(initial[12]))
        check_view()
        journal.write('baseline', arm_head_q=initial.tolist())
        if not args.motion_mode:
            print('Read-only checks passed; no command sent.')
            return 0

        from robot.config import R1A7Config
        from robot.arm_control import RealArmController
        from teleop.real_controller import RealTeleopController

        class WristTestController(RealTeleopController):
            def _create_initial_q(self):
                if hasattr(self, 'test_start_q'):
                    return self.test_start_q.copy()
                return super()._create_initial_q()

            def _output_command(self):
                # Exactly 14 arm commands; no hand or head target updates.
                command = self.test_initial[:14].copy()
                command[7:] = self.q[self.right_arm_q_indices]
                self.arm_ctrl.ctrl_dual_arm(command)

        ctrl = WristTestController(R1A7Config(reset_blend_frames=100), motion_mode=False,
                                   enable_real_hand=False, mock_xr=True, visualize=False)
        ctrl.test_initial = initial.copy()
        for i, (name, _, _, _) in enumerate(probe.real_joints):
            ctrl.q[ctrl.joint_indices[name]] = initial[i]
        ctrl.test_start_q = ctrl.q.copy()
        # Verify limits using the existing model; the test never requests IK.
        index = ctrl.joint_indices['right_wrist_pitch_joint']
        target = initial[12] - .02
        if not ctrl.model.lowerPositionLimit[index] + .05 < target < ctrl.model.upperPositionLimit[index] - .05:
            raise Halt('wrist test target too near a limit')
        probe.require_fresh()
        probe.require_quiet_commands()
        if np.max(np.abs(probe.arm_head_positions() - initial)) > .01:
            raise Halt('state changed before takeover')
        check_view()
        show('HOLD BASELINE')
        ctrl.arm_ctrl = RealArmController(ctrl.config, args.net, initial_command_q=initial,
                                          skip_motion_switcher=True)
        ctrl.motion_mode = True

        def watch():
            check_view()
            probe.require_fresh()
            with sub.lock:
                if time.time_ns() - sub.received_at_ns > 500_000_000:
                    raise Halt('camera stream stopped')
            measured = probe.arm_head_positions()
            show(measured=float(measured[12]))
            untouched = list(range(12)) + [13, 14, 15]
            if np.max(np.abs(measured[untouched] - initial[untouched])) > .025:
                raise Halt('an untested joint moved more than 0.025 rad')
            if abs(measured[12] - initial[12]) > .04:
                raise Halt('right wrist excursion exceeds test bound')
            return measured

        # Stabilize the initial hold before requesting the ONE small action.
        for _ in range(25):
            watch()
            time.sleep(.02)
        ctrl.q[index] = target
        show('MOVE -0.020 rad', target=float(target))
        ctrl._output_command()
        journal.write('single_target', joint='right_wrist_pitch_joint', start=float(initial[12]), target=float(target))
        deadline = time.monotonic() + 3
        stable = 0
        while time.monotonic() < deadline:
            measured = watch()
            journal.write('feedback', wrist_pitch=float(measured[12]))
            stable = stable + 1 if abs(measured[12] - target) < .005 else 0
            if stable >= 10:
                break
            time.sleep(.02)
        else:
            raise Halt('right wrist did not follow the 0.02 rad command')
        reached = float(measured[12])
        # Fresh observation after the small movement, before returning.
        frames, fid, _ = fresh_frames(sub, fid, time.time_ns(), watchdog=watch)
        journal.images(1, selected_views(frames, 'head') + selected_views(frames, 'wrist'))
        show('RETURN / 100 frames', target=float(initial[12]))
        ctrl.reset()  # shared joint-linear blend, target is this test's start
        while ctrl._reset_blending:
            watch()
            ctrl.advance_home_blend()
            time.sleep(.02)
        deadline = time.monotonic() + 3
        stable = 0
        while time.monotonic() < deadline:
            measured = watch()
            stable = stable + 1 if abs(measured[12] - initial[12]) < .005 else 0
            if stable >= 10:
                break
            time.sleep(.02)
        else:
            raise Halt('right wrist did not return to the test start')
        frames, _, _ = fresh_frames(sub, fid, time.time_ns(), watchdog=watch)
        journal.images(2, selected_views(frames, 'head') + selected_views(frames, 'wrist'))
        journal.write('passed', start=float(initial[12]), reached=reached, returned=float(measured[12]))
        success = True
        show('PASS / RETURNED', reason='Small motion and feedback verified. Returned to start. No grasp performed.')
        print(f'PASS: wrist pitch {initial[12]:.5f} -> {reached:.5f} -> {measured[12]:.5f} rad')
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        show('HALT', reason=str(exc) or 'interrupted')
        journal.write('halt', reason=str(exc))
        print('HALT:', str(exc) or 'interrupted')
        return 2
    finally:
        if ctrl is not None and ctrl.motion_mode and not success:
            # Cancel the outstanding right-arm target and keep the existing
            # writer alive. Failure must not trigger a blind automatic return.
            real = ctrl.arm_ctrl.real
            with real.lock:
                command = real.command_q[:14].copy()
            ctrl.arm_ctrl.ctrl_dual_arm(command)
            ctrl.q[ctrl.right_arm_q_indices] = command[7:14]
            show('HOLD', target=float(command[12]))
            print('Test failed: holding current commands; no automatic return. Log:', journal.path, flush=True)
            while True:
                try:
                    choice = input('Check scene; enter return to blend back to test start: ').strip()
                    if choice != 'return':
                        continue
                    probe.require_fresh()
                    fresh_frames(sub, 0, time.time_ns())
                    ctrl.reset()
                    while ctrl._reset_blending:
                        probe.require_fresh()
                        with sub.lock:
                            if time.time_ns() - sub.received_at_ns > 500_000_000:
                                raise Halt('camera stream stopped during recovery')
                        ctrl.advance_home_blend()
                        time.sleep(.02)
                    if np.max(np.abs(probe.arm_head_positions()[7:14] - initial[7:14])) > .01:
                        raise Halt('return not confirmed by feedback')
                    journal.write('operator_return_complete')
                    break
                except EOFError:
                    time.sleep(.2)
                except KeyboardInterrupt:
                    print('Holding. Recovery requires checking the robot/scene before further motion.', flush=True)
                except Exception as exc:
                    with real.lock:
                        command = real.command_q[:14].copy()
                    ctrl.arm_ctrl.ctrl_dual_arm(command)
                    ctrl.q[ctrl.right_arm_q_indices] = command[7:14]
                    print('Recovery stopped; holding:', exc, flush=True)
        if ctrl is not None:
            ctrl._on_stop()
        if probe is not None:
            probe.close()
        sub.close()
        print('Log:', journal.path)


if __name__ == '__main__':
    raise SystemExit(main())
