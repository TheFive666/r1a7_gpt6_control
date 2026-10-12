"""Right-only adapter around the existing FK, IK, DDS and home blend code."""
from __future__ import annotations

import time
from collections import deque
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation as R

from robot.arm_model import clamp_configuration, link_pose
from robot.config import (DEX1_SIM_OPEN, DEX1_SIM_CLOSE, LEFT_ARM_JOINTS,
                          RIGHT_ARM_JOINTS, LEFT_DEX1_JOINTS, RIGHT_DEX1_JOINTS)
from agent.astra_policy import Halt
from agent.step_interpolation import CONTROL_DT, joint_step_samples
from agent.settling_feedback import partial_response
from teleop.placo_solver import PlacoIKSolver
from teleop.real_controller import RealTeleopController


class CollisionGuard:
    """URDF self-collision checks; external scene clearance is a visual gate.

    Ignore only same-body and directly connected bodies. Do not learn away
    collisions from the initial configuration. Test the small joint segment
    before sending its ONE target; these samples are never motion commands.
    """
    def __init__(self, controller):
        self.model = controller.model
        self.data = self.model.createData()
        self.geom = pin.buildGeomFromUrdf(
            self.model, str(controller.urdf_path), pin.GeometryType.COLLISION,
            package_dirs=[str(controller.urdf_path.parent), str(controller.config.urdf_path.rsplit('/', 1)[0])])
        for i, a in enumerate(self.geom.geometryObjects):
            for j in range(i + 1, len(self.geom.geometryObjects)):
                b = self.geom.geometryObjects[j]
                pa, pb = a.parentJoint, b.parentJoint
                if pa == pb or self.model.parents[pa] == pb or self.model.parents[pb] == pa:
                    continue
                self.geom.addCollisionPair(pin.CollisionPair(i, j))
        if not self.geom.collisionPairs:
            raise Halt("URDF collision geometry unavailable")
        self.gdata = pin.GeometryData(self.geom)

    def check(self, start, end):
        for w in np.linspace(0.0, 1.0, 9):
            q = start + w * (end - start)
            if pin.computeCollisions(self.model, self.data, self.geom, self.gdata, q, False):
                names = []
                for pair, result in zip(self.geom.collisionPairs, self.gdata.collisionResults):
                    if result.isCollision():
                        names.append(self.geom.geometryObjects[pair.first].name + '/' + self.geom.geometryObjects[pair.second].name)
                raise Halt("URDF self-collision risk: " + ', '.join(names))


class RightOnlyController(RealTeleopController):
    def __init__(self, config, snapshot, workspace, *, position_checks=True):
        # Build model/IK via the existing real-controller class, but defer ALL
        # real DDS writers until after handover, geometry and credential checks.
        super().__init__(config, motion_mode=False, enable_real_hand=False,
                         mock_xr=True, visualize=False)
        self.ready = False
        self.right_ratio = float(snapshot.get("right_closed_ratio", 0.0))
        if not np.isfinite(self.right_ratio) or not 0<=self.right_ratio<=1:
            raise Halt("invalid handed-over right gripper command")
        self.left_ratio = float(snapshot['left_closed_ratio'])
        if not np.isfinite(self.left_ratio) or not 0 <= self.left_ratio <= 1:
            raise Halt("invalid handed-over left gripper command")
        joints = snapshot['joint_commands']
        for name in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + ['head_pitch_joint', 'head_yaw_joint', 'waist_yaw_joint']:
            self.q[self.joint_indices[name]] = float(joints[name])
        for name in LEFT_DEX1_JOINTS:
            self.q[self.joint_indices[name]] = DEX1_SIM_OPEN + self.left_ratio * (DEX1_SIM_CLOSE - DEX1_SIM_OPEN)
        for name in RIGHT_DEX1_JOINTS:
            self.q[self.joint_indices[name]] = DEX1_SIM_OPEN
        if not np.all(np.isfinite(self.q)) or not np.array_equal(self.q, clamp_configuration(self.model, self.q.copy())):
            raise Halt("handover contains invalid/out-of-limit joints")
        self.frozen = self.q.copy()
        self.waist_command = float(self.q[self.joint_indices['waist_yaw_joint']])
        self.home_right = super()._create_initial_q()[self.right_arm_q_indices].copy()
        self.workspace = np.asarray(workspace, dtype=float).reshape(3, 2)
        self.position_checks = bool(position_checks)
        self.initial_command = np.array([joints[n] for n in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + ['head_pitch_joint', 'head_yaw_joint']], dtype=float)
        self.fixed_indices = [i for i in range(self.model.nq) if i not in self.right_arm_q_indices]
        # Only create a right frame task. Inactive joints are also restored
        # after solving; no Placo proposal can ever change the other arm.
        try:
            self.placo_solver = PlacoIKSolver(self.urdf_path, self.q, {'right': self.ee_frames['right']},
                                             self.joint_indices, hold_joints=tuple(n for n in self.joint_indices
                                                 if n not in RIGHT_ARM_JOINTS and n != 'waist_yaw_joint'))
            # This discrete action must satisfy its EEF target without borrowing
            # motion from joints the right-only publisher never changes.
            self.placo_solver.effector_task['right'].configure('right', 'hard', 1.0)
            self.placo_solver.hold_task.configure('hold', 'hard', 1.0)
            self.placo_solver.waist_hold_task.configure('waist_hold', 'hard', 10.0)
            self.use_placo = True
        except Exception as exc:
            raise Halt(f"Placo initialization failed: {type(exc).__name__}") from exc
        self.collisions = CollisionGuard(self) if self.position_checks else None
        if self.position_checks:
            self.collisions.check(self.q, self.q)
            self.check_workspace(self.pose()[0])
        root = ET.parse(config.urdf_path).getroot()
        self.tool_geometry = {
            'ee_frame': 'right_dex1_base_link',
            'base_frame': self.model.frames[1].name,
            'finger_joints': [{
                'name': j.attrib['name'], 'axis_local': j.find('axis').attrib['xyz'],
                'origin_xyz': j.find('origin').attrib['xyz'],
                'origin_rpy': j.find('origin').attrib['rpy'],
                'open_m': DEX1_SIM_OPEN,
            } for j in root.findall('joint') if j.attrib['name'] in RIGHT_DEX1_JOINTS],
            'note': 'Fingers slide along local +/-Y. Gripper extends along local +X. Finger link origins are NOT fingertips; use visible finger surfaces for clearance.'}

    def attach_hardware(self, interface, probe, held_pid=None):
        from robot.arm_control import RealArmController
        from robot.eef.dex1 import Dex1
        probe.require_fresh()
        if held_pid is None:
            probe.require_quiet_commands()
        else:
            from pathlib import Path
            proc = Path(f'/proc/{held_pid}/cmdline').read_bytes()
            if not any(name in proc for name in (b'scripts/astra_direct_rollout.py',b'scripts/right_arm_wave.py',b'agp/server_r1a7.py',b'scripts/fixed_cake_rollout.py')):
                raise Halt('handover requires known Direct holding process')
            with probe.lock:
                held = probe.messages.get('lowcmd')
                values = None if held is None else np.array([held.motor_cmd[i].q for _,i,_,_ in probe.real_joints])
            if values is None:
                raise Halt('holding command unavailable before prepared handover')
            handover_error = float(np.max(np.abs(values - self.initial_command)))
            if handover_error > 1e-7:
                raise Halt(f'holding command changed before prepared handover: {handover_error:.6f} rad')
        measured = probe.arm_head_positions()
        if self.position_checks and np.max(np.abs(measured - self.initial_command)) > 0.08:
            raise Halt("handover differs from measured arm/head state by >0.08 rad")
        # Already in debugging mode: never call ReleaseMode or switch modes.
        self.arm_ctrl = RealArmController(self.config, interface, initial_command_q=self.initial_command,
                                          skip_motion_switcher=True)
        self.motion_mode = True
        self.ready = True
        # Keep diagnostics out of the time-critical writer (stdout may block).
        self.arm_ctrl.real.config.report_period = float('inf')
        if held_pid is not None:
            import os, signal
            # Both writers briefly emit identical holding commands. Retire the
            # old writer ONLY after this writer has successfully published.
            deadline = time.monotonic() + 2
            while self.arm_ctrl.real.last_publish_time is None:
                if time.monotonic() > deadline:
                    raise Halt('replacement writer not publishing; old holder retained')
                time.sleep(.005)
            probe.require_fresh()
            with probe.lock:
                held = probe.messages['lowcmd']
                values = np.array([held.motor_cmd[i].q for _,i,_,_ in probe.real_joints])
            if not np.allclose(values, self.initial_command, atol=1e-7, rtol=0):
                raise Halt('holding command changed; no handover motion allowed')
            dedicated_group = os.getpgid(held_pid) == held_pid and os.getpgrp() != held_pid
            if dedicated_group:
                # Retire the dedicated old server's forked lock holders only
                # after this replacement writer has published identical hold.
                os.killpg(held_pid, signal.SIGTERM)
            else:
                os.kill(held_pid, signal.SIGTERM)
            deadline = time.monotonic() + 2
            from pathlib import Path
            while Path(f'/proc/{held_pid}').exists():
                if time.monotonic() > deadline:
                    raise Halt('old writer did not exit; hold only')
                time.sleep(.01)
            if dedicated_group:
                # Python's resource tracker ignores SIGTERM and can inherit
                # the owner fd. Parent is gone; the replacement holds hardware.
                try:
                    os.killpg(held_pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        try:
            self.hand_ctrl = Dex1(initial_ratios=(self.left_ratio, self.right_ratio), publish_left=False)
            self._output_command()
        except BaseException:
            self.freeze()
            raise

    def pose(self, q=None):
        return link_pose(self.model, self.data, self.q if q is None else q, 'right_dex1_base_link')

    def refresh(self):
        if not self.motion_mode:
            return
        actual = self.arm_ctrl.get_current_dual_arm_q()
        if not np.all(np.isfinite(actual)):
            raise Halt("non-finite joint feedback")
        # Match Pico teleop: q remains the persistent commanded configuration.
        # Store feedback separately so static load error does not erase the
        # accumulated command before the next visual EE increment.
        self.measured_q = self.q.copy()
        self.measured_q[self.left_arm_q_indices] = actual[:7]
        self.measured_q[self.right_arm_q_indices] = actual[7:14]

    def measured_pose(self):
        self.refresh()
        return self.pose(getattr(self, 'measured_q', self.q))

    def check_workspace(self, p):
        if not self.position_checks:
            return
        if not np.all(np.isfinite(p)) or np.any(p < self.workspace[:, 0]) or np.any(p > self.workspace[:, 1]):
            raise Halt("right EE outside configured base-frame workspace")

    def _validated_solution(self, raw, start, position, quat, position_tolerance, rotation_tolerance):
        if raw is None or np.shape(raw) != np.shape(start) or not np.all(np.isfinite(raw)):
            raise Halt("IK returned no finite solution")
        q = start.copy()
        idx = self.right_arm_q_indices
        q[idx] = raw[idx]
        clamped = clamp_configuration(self.model, q.copy())
        if self.position_checks:
            if not np.array_equal(q, clamped):
                raise Halt("IK hit joint limits")
            lo, hi = self.model.lowerPositionLimit[idx], self.model.upperPositionLimit[idx]
            if np.any(q[idx] <= lo + 0.015) or np.any(q[idx] >= hi - 0.015):
                raise Halt("IK too near a joint limit")
        # Do NOT reject large per-target joint deltas here. A small EE step can
        # still need >0.1 rad of joint change near singularities; smoothness is
        # already enforced by low-level max_joint_delta=0.04 rad / 20 ms.
        # Still halt on: IK fail, joint limits, residual, workspace, collision.
        p, r = self.pose(q)
        # This is the numerical IK residual, before any hardware tracking error.
        # A solver must actually reach the requested EEF target in the model.
        position_error = float(np.linalg.norm(p - position))
        rotation_error = float((R.from_quat(r).inv() * R.from_quat(quat)).magnitude())
        if position_error > position_tolerance or rotation_error > rotation_tolerance:
            raise Halt(f"IK residual too large: position={position_error:.6f}m rotation={rotation_error:.6f}rad")
        self.check_workspace(p)
        if self.position_checks:
            self.collisions.check(start, q)
        return clamped if self.position_checks else q

    def solve_step(self, decision):
        self.refresh()
        start = self.q.copy()
        p, quat = self.pose(start)
        position = p + np.array(decision.translation)
        rotation = R.from_rotvec(decision.rotation) * R.from_quat(quat)
        self.check_workspace(position)
        # Reject inaccurate/no-progress solutions even for submillimetre steps.
        distance = float(np.linalg.norm(decision.translation))
        angle = float(np.linalg.norm(decision.rotation))
        ptol = min(0.0015, max(0.0001, distance * 0.25))
        rtol = min(0.01, max(0.001, angle * 0.25))
        # Teleop repeatedly solves a persistent target. Converge that same
        # target here before publishing, without accumulating the delta again.
        candidate = start.copy()
        for iteration in range(20):
            try:
                raw = self.placo_solver.solve(candidate, {'right': (position, rotation.as_quat())})
            except Exception as exc:
                raise Halt(f"Placo IK failed: {type(exc).__name__}") from exc
            try:
                return self._validated_solution(raw, start, position, rotation.as_quat(), ptol, rtol)
            except Halt as exc:
                if not str(exc).startswith('IK residual') or iteration == 19:
                    raise
            candidate = start.copy()
            candidate[self.right_arm_q_indices] = raw[self.right_arm_q_indices]

    def _output_command(self):
        # The shared home blend multiplies even equal endpoints, introducing
        # roundoff. Restore their exact stored values before publishing.
        if not np.allclose(self.q[self.fixed_indices], self.frozen[self.fixed_indices], rtol=0, atol=1e-14):
            raise Halt("inactive joint command changed")
        self.q[self.fixed_indices] = self.frozen[self.fixed_indices]
        arms = np.concatenate((self.frozen[self.left_arm_q_indices], self.q[self.right_arm_q_indices]))
        self.arm_ctrl.ctrl_dual_arm(arms)  # exactly 14: unchanged left 7 + new right 7
        if self.hand_ctrl is not None:
            self.hand_ctrl.set_gripper_ratios(self.left_ratio, getattr(self, "right_ratio", 0.0))

    def execute_one(self, q, watchdog, timeout=3.0, position_tolerance_m=None, progress_callback=None):
        start_p, start_r = self.pose()
        target_p, target_r = self.pose(q)
        measured_start_p, measured_start_r = self.measured_pose()
        expected_delta = target_p - start_p
        expected_rotation = R.from_quat(target_r) * R.from_quat(start_r).inv()
        # Match the teleop path: settle is judged by the emitted joint command
        # tracking and the achieved translation.  The IK solution can introduce
        # a small orientation residual even for a pure translation; treating
        # that residual as a hard failure makes SDK motion diverge from the
        # working teleop interface.
        position_tolerance = max(0.001, min(0.003, np.linalg.norm(expected_delta) * 0.15))
        if position_tolerance_m is not None:
            if not np.isfinite(position_tolerance_m) or not 0 < position_tolerance_m <= 0.010:
                raise ValueError("position tolerance must be in (0, 0.010] m")
            position_tolerance = float(position_tolerance_m)
        rotation_tolerance = max(0.04, expected_rotation.magnitude() * 0.50)
        samples = joint_step_samples(self.q, q, self.right_arm_q_indices)
        # Validate the entire interpolation before publishing its first sample.
        previous = self.q.copy()
        if self.position_checks:
            for sample in samples:
                self.check_workspace(self.pose(sample)[0])
                if not np.array_equal(sample, clamp_configuration(self.model, sample.copy())):
                    raise Halt('interpolation violates joint limits')
                self.collisions.check(previous, sample)
                previous = sample
        if not self.motion_mode:
            self.q = q.copy()
            self._output_command()  # Simulation/mock adapter only; no real writer.
            return
        target = q[self.right_arm_q_indices].copy()
        stable_since = None
        deadline = time.monotonic() + len(samples) * CONTROL_DT + timeout
        tick = time.monotonic()
        sample_index = 0
        publish_times = []
        settled_positions = deque()
        self.last_execution = {}
        while time.monotonic() < deadline:
            watchdog()
            if progress_callback is not None:
                progress_callback(0.9 * sample_index / max(1, len(samples)))
            actual = self.arm_ctrl.get_current_dual_arm_q()[7:]
            measured_q = self.q.copy()
            measured_q[self.right_arm_q_indices] = actual
            measured_p, measured_r = self.pose(measured_q)
            measured_rotation = R.from_quat(measured_r) * R.from_quat(measured_start_r).inv()
            pose_settled = (np.linalg.norm((measured_p - measured_start_p) - expected_delta) <= position_tolerance and
                            (measured_rotation.inv() * expected_rotation).magnitude() <= rotation_tolerance)
            with self.arm_ctrl.real.lock:
                command = self.arm_ctrl.real.command_q[7:14].copy()
            if sample_index == len(samples) and np.max(np.abs(command - target)) < 1e-6:
                settled_positions.append((time.monotonic(), measured_p.copy()))
                while settled_positions and time.monotonic() - settled_positions[0][0] > 0.6:
                    settled_positions.popleft()
            # During a rate-limited transition, compare feedback with the
            # command actually emitted, not the still-unreached final target.
            # The low-level DDS writer already rate-limits every joint step.
            # Publishing the target alone does not establish EEF movement.
            if (sample_index == len(samples) and
                    np.max(np.abs(command - target)) < 1e-6 and pose_settled):
                stable_since = time.monotonic() if stable_since is None else stable_since
                if time.monotonic() - stable_since >= 0.25:
                    self.partial_count = 0
                    self.refresh()
                    gaps = np.diff(publish_times)
                    self.last_execution = dict(samples=len(publish_times),
                        outcome='reached', measured_delta_m=(measured_p-measured_start_p).tolist(),
                        effective_hz=float(1 / np.mean(gaps)) if len(gaps) else 0,
                        max_gap_ms=float(np.max(gaps)*1000) if len(gaps) else 0)
                    return
            else:
                stable_since = None
            # One bounded action, sampled at 30 Hz. No future policy target
            # is queued; after completion the caller must obtain fresh images.
            if sample_index < len(samples):
                self.q = samples[sample_index].copy()
                sample_index += 1
            self._output_command()
            publish_times.append(time.monotonic())
            gaps = np.diff(publish_times)
            self.last_execution = dict(samples=len(publish_times),
                effective_hz=float(1/np.mean(gaps)) if len(gaps) else 0.,
                max_gap_ms=float(np.max(gaps)*1000) if len(gaps) else 0.,
                position_error_mm=float(np.linalg.norm((measured_p-measured_start_p)-expected_delta)*1000),
                rotation_error_rad=float((measured_rotation.inv()*expected_rotation).magnitude()))
            tick += CONTROL_DT
            now = time.monotonic()
            if tick < now:
                tick = now + CONTROL_DT  # Never catch up with a command burst.
            time.sleep(max(0.0, tick - now))
        span = (max(np.linalg.norm(p - settled_positions[0][1]) for _, p in settled_positions)
                if len(settled_positions) >= 10 else float('inf'))
        partial = (self.right_ratio < 0.05 and len(settled_positions) >= 10
                   and settled_positions[-1][0]-settled_positions[0][0] >= 0.45
                   and expected_rotation.magnitude() < 1e-8
                   and partial_response(expected_delta, measured_p-measured_start_p,
                       float((measured_rotation.inv()*expected_rotation).magnitude()),
                       float(np.max(np.abs(actual-command))), span))
        if partial:
            self.partial_count = getattr(self, 'partial_count', 0) + 1
            if self.partial_count <= 2:
                self.last_execution.update(outcome='partial', requires_reobserve=True,
                    measured_delta_m=(measured_p-measured_start_p).tolist(),
                    requested_delta_m=expected_delta.tolist())
                self.refresh()
                return
        raise Halt("right arm failed to settle; no subsequent motion allowed; " + str(self.last_execution))

    def freeze(self):
        if not self.ready:
            return
        # Cancel any pending target, including a partially completed action.
        # The low-level writer keeps holding; do not return home on failure.
        real = self.arm_ctrl.real
        with real.lock:
            frozen_right = real.command_q[7:14].copy()
        self.q[:] = self.frozen
        self.q[self.right_arm_q_indices] = frozen_right
        self._output_command()

    def _create_initial_q(self):
        # reset()/advance_home_blend() use the existing 100-frame linear blend,
        # but its goal must restore only the right arm.
        if not hasattr(self, 'frozen'):
            return super()._create_initial_q()
        goal = self.frozen.copy()
        goal[self.right_arm_q_indices] = self.home_right
        return goal

    def return_home(self, watchdog):
        self.freeze()
        start = self.q.copy()
        goal = self._create_initial_q()
        self.collisions.check(start, goal)
        # Also validate the EE workspace throughout the existing blend.
        for w in np.linspace(0, 1, self.reset_blend_frames + 1):
            self.check_workspace(self.pose(start + w * (goal - start))[0])
        self.reset()
        while self._reset_blending:
            watchdog()
            self.advance_home_blend()
            time.sleep(0.02)
        self.execute_one(self.q, watchdog)
