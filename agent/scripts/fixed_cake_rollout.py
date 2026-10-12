#!/usr/bin/env python3
"""Run the fixed cake-to-plate trajectory through the teleop IK path."""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

REPO_ROOT = Path(__file__).resolve().parents[2]
ROOT = REPO_ROOT
sys.path.insert(0, str(ROOT))

from robot.config import R1A7Config
from agent.r1a7_sdk import ExternalVideoRecorder, SDKError
from agent.scripts.astra_grasp_rollout import ReadOnlyDDS
from teleop.image_client import FourCameraSubscriber
from agent.astra_policy import Halt
from agent.astra_robot import RightOnlyController


def plan():
    steps = []

    def move(count, stage, dpos, pitch, text, observation):
        for index in range(1, count + 1):
            steps.append(dict(action="move", stage=stage, dpos=dpos, pitch=pitch,
                              text=f"{text}（{index}/{count}）", observation=observation))

    def translation(distance, axis, stage, label):
        remaining = abs(distance)
        while remaining > 1e-9:
            amount = min(0.03, remaining)
            delta = [0.0, 0.0, 0.0]
            delta[axis] = float(np.copysign(amount, distance))
            move(1, stage, delta, 0, f"{label}{amount * 100:g}cm", "等待新图复核实际位置")
            remaining -= amount

    translation(.06, 2, "approach", "向上移动")
    translation(.16, 0, "approach", "向前移动")
    move(6, "capture", (0, 0, 0), -5, "Pitch向下旋转5度", "等待新图复核指尖位置")
    steps.append(dict(action="gripper", stage="close", ratio=1.0,
                      text="闭合夹爪抓取蛋糕", observation="须从新图确认蛋糕位于指间"))
    translation(.10, 2, "raise", "向上移动")
    translation(.20, 1, "transfer", "向左移动")
    translation(-.06, 2, "lower", "向下移动")
    steps.append(dict(action="gripper", stage="release", ratio=0.0,
                      text="打开夹爪释放蛋糕", observation="须确认蛋糕在盘内受支撑"))
    return steps


def show(view_dir, state, **values):
    state.update(values)
    view_dir.mkdir(parents=True, exist_ok=True)
    temp = view_dir / "fixed_status.tmp"
    temp.write_text(json.dumps(state, ensure_ascii=False))
    temp.replace(view_dir / "status.json")


def snapshot(probe):
    with probe.lock:
        state = probe.messages["lowstate"]
        command = probe.messages.get("lowcmd")
        joints = {name: float((command.motor_cmd[index] if command else state.motor_state[index]).q)
                  for name, index, _, _ in probe.real_joints}
        joints["waist_yaw_joint"] = float(state.motor_state[13].q)
        left = float(probe.messages["left_state"].states[0].q)
        right_cmd = probe.messages.get("right_cmd")
        right = float((right_cmd.cmds[0] if right_cmd else probe.messages["right_state"].states[0]).q)
    return dict(joint_commands=joints,
                left_closed_ratio=float(np.clip(1 - left / 5, 0, 1)),
                right_closed_ratio=float(np.clip(1 - right / 5, 0, 1)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--net", default="enp5s0")
    parser.add_argument("--view-dir", type=Path, default=ROOT / "logs/agp_r1a7_view")
    parser.add_argument("--motion-mode", action="store_true")
    parser.add_argument("--enable-dex1", action="store_true")
    parser.add_argument("--held-pid", type=int)
    parser.add_argument("--external-camera", default="/dev/video0")
    parser.add_argument("--print-plan", action="store_true")
    parser.add_argument("--review-steps", action="store_true")
    parser.add_argument("--start-step", type=int, default=1)
    parser.add_argument("--position-tolerance-mm", type=float, default=None)
    parser.add_argument("--plan-file", type=Path)
    args = parser.parse_args()
    steps = json.loads(args.plan_file.read_text()) if args.plan_file else plan()
    if args.print_plan:
        print(json.dumps(steps, ensure_ascii=False, indent=2))
        return 0
    if args.motion_mode != args.enable_dex1:
        parser.error("真机执行需要同时指定 --motion-mode --enable-dex1")

    probe = ReadOnlyDDS(args.net)
    controller = None
    recorder = None
    sub = FourCameraSubscriber("192.168.123.164", 5555)
    sub.start()
    ui = dict(iteration=0, max_steps=len(steps), stage="approach", planning_phase="observing",
              right_gripper_ratio=0.0, inference_total_s=0.0)
    try:
        deadline = time.monotonic() + 5
        while True:
            try:
                probe.require_fresh()
                break
            except Halt:
                if time.monotonic() > deadline:
                    raise
                time.sleep(0.05)

        # Fixed operator trajectory: no workspace, joint-position or collision-position rejection.
        controller = RightOnlyController(R1A7Config(), snapshot(probe),
                                         [-1, 1, -1, 1, -1, 1], position_checks=False)
        if args.start_step >= 17 and steps[args.start_step - 1]["stage"] in ("raise", "transfer", "lower", "release"):
            controller.right_ratio = 1.0
        controller.last_execution = {}
        if args.motion_mode:
            controller.attach_hardware(args.net, probe, held_pid=args.held_pid)
            recorder = ExternalVideoRecorder(
                ROOT / "logs" / ("fixed_external_" + time.strftime("%Y%m%d_%H%M%S")),
                device=args.external_camera)
            try:
                recorder.start()
            except SDKError as exc:
                if "Device or resource busy" not in str(exc):
                    raise
                recorder = None

        def watchdog():
            probe.require_fresh()
            if (args.view_dir / "stop_requested").exists():
                raise Halt("operator STOP")
            alive = args.view_dir / "view_alive"
            if not alive.exists() or time.time() - alive.stat().st_mtime > 3:
                raise Halt("viewer stale")
            if controller.ready and not controller.arm_ctrl.real.writer_thread.is_alive():
                raise Halt("DDS writer stopped")

        def observe(number):
            after = time.time_ns()
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                watchdog()
                frames, fid, stamp = sub.get()
                if frames is not None and stamp > after:
                    for name, index in (("top", 1), ("wrist", 3)):
                        cv2.imwrite(str(args.view_dir / f"step_{number:02d}_{name}.jpg"), frames[index])
                    return
                time.sleep(.02)
            raise Halt("no fresh post-action images")

        last_delta = None
        for number, step in enumerate(steps, 1):
            if number < args.start_step:
                continue
            rotvec = Rotation.from_euler("y", step.get("pitch", 0), degrees=True).as_rotvec()
            last_delta = dict(translation_m=[0, 0, 0], rotvec_rad=[0, 0, 0])
            if step["action"] == "move":
                last_delta = dict(translation_m=step["dpos"], rotvec_rad=rotvec.tolist())
            show(args.view_dir, ui, iteration=number, stage=step["stage"], planning_phase="ready",
                 action_prompt=step["text"], observation_text=step["observation"],
                 planned_delta_pose=last_delta, delta_status="待下发")
            observe(number - 1)
            if args.review_steps:
                print(f"REVIEW step {number}: {step['text']}", flush=True)
                approval = args.view_dir / f"approve_{number:02d}"
                while not approval.exists():
                    watchdog()
                    time.sleep(.05)
                approval.unlink()
            started = time.monotonic()
            show(args.view_dir, ui, planning_phase="executing", execution_progress=0.0)
            if step["action"] == "move":
                position, quaternion = controller.pose()
                target_position = position + np.asarray(step["dpos"], dtype=float)
                target_quaternion = (Rotation.from_rotvec(rotvec) *
                                     Rotation.from_quat(quaternion)).as_quat()
                solved = controller.placo_solver.solve(
                    controller.q, {"right": (target_position, target_quaternion)})
                target = controller.q.copy()
                target[controller.right_arm_q_indices] = solved[controller.right_arm_q_indices]
                def progress(value):
                    show(args.view_dir, ui, execution_progress=value, execution_s=time.monotonic() - started)
                controller.execute_one(target, watchdog, timeout=5,
                    progress_callback=progress,
                    position_tolerance_m=None if args.position_tolerance_mm is None else args.position_tolerance_mm / 1000)
            else:
                controller.right_ratio = step["ratio"]
                if controller.hand_ctrl:
                    controller.hand_ctrl.set_gripper_ratios(controller.left_ratio, controller.right_ratio)
                if args.motion_mode:
                    for tick in range(20):
                        watchdog()
                        show(args.view_dir, ui, execution_progress=(tick + 1) / 20, execution_s=time.monotonic() - started)
                        time.sleep(.05)
            observe(number)
            if controller.last_execution.get("outcome") == "partial" and step["action"] == "move":
                raise Halt("partial movement; trajectory needs replanning")
            with (args.view_dir / "steps.jsonl").open("a") as log:
                log.write(json.dumps(dict(step=number, plan=step, feedback=controller.last_execution)) + "\n")
            show(args.view_dir, ui, planning_phase="done", execution_progress=1.0,
                 execution_s=time.monotonic() - started, delta_status="已下发并执行",
                 right_gripper_ratio=controller.right_ratio)
            print(f"Step {number:02d}/{len(steps)}：{step['text']}", flush=True)

        show(args.view_dir, ui, stage="verify", action_prompt="固定轨迹执行完成",
             observation_text="轨迹执行结束，等待新图确认放置结果", delta_status="待视觉验收")
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        if controller and controller.ready:
            controller.freeze()
        show(args.view_dir, ui, planning_phase="hold", delta_status=f"执行异常：{exc}",
             action_prompt=f"轨迹暂停：{exc}")
        print(f"HOLD: {exc}", flush=True)
        return 2
    finally:
        if recorder is not None:
            recorder.stop()
        if controller and controller.ready:
            controller.freeze()
            print("Holding current command; process remains alive.", flush=True)
            while True:
                time.sleep(1)
        sub.close()
        probe.close()


if __name__ == "__main__":
    raise SystemExit(main())
