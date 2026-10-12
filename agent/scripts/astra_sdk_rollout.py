#!/usr/bin/env python3
"""GPT-6 Astra rollout using only the public R1-A7 SDK control surface."""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
from scipy.spatial.transform import Rotation

REPO_ROOT = Path(__file__).resolve().parents[2]
ROOT = REPO_ROOT
sys.path.insert(0, str(ROOT))

from agent.r1a7_sdk import (ACTION_SCHEMA, Action, ControlLedger, EpisodeTrace, ExternalVideoRecorder,
                       load_reference,
                       R1A7PickPlaceSDK, SDKError)  # noqa: E402
from agent.astra_policy import AstraClient, Halt  # noqa: E402


PROMPT = """Control only the R1-A7 right arm and Dex1: pick yellow cake into blue plate.
Return one schema action from fresh head_right and wrist_right RGB. Chinese concise
observation/reason. Base +X forward, +Y left, +Z up. Every move translates 20 mm
and rotates <=0.025 rad. Preserve orientation when possible. Never output joints.
Use current ledger gate. Choose purposeful movement immediately when safe; no
redundant observe. Never enlarge a step if its full swept path is not clear.
Close only when BOTH finger contact surfaces overlap opposite sides of the cake
at grasp height, not merely when cake projects between widely spread fingers.
After close, first move is pure +Z 20 mm. Compare before/after images to verify
cake follows before transport. Empty failed grasp: open only with visible empty
fingers and cake supported on table; then correct alignment before retrying.
Verified grasp: keep closed and move only pure +Z 20mm in stage raise until
lift_state.transfer_allowed is true: measured EEF Z must rise at least 0.15m
above its height at closure. Command sums do not prove actual lift. Only then
transfer over blue plate and lower. Inspect clearance on every fresh observation.
Open only with fresh support inside plate. Done only after fresh post-release
images show stable cake inside plate and clear fingers. If uncertain or blocked,
stop and explain. Commanded and measured poses differ: use actual feedback and
fresh images to replan. Historical demonstrations are context, never evidence.
"""


def planner_views(images, phase, approach_edge=960):
    """Bound image upload size only in free approach; retain contact detail."""
    if phase != "approach":
        return images
    result = []
    for name, frame in images:
        height, width = frame.shape[:2]
        scale = min(1.0, approach_edge / max(height, width))
        result.append((name, cv2.resize(frame, (max(1, round(width * scale)),
                                               max(1, round(height * scale))),
                                       interpolation=cv2.INTER_AREA) if scale < 1 else frame))
    return result


def planner_state(state):
    # Keep commanded/measured poses distinct; joint arrays and static URDF
    # geometry remain in the full trace rather than every planning request.
    return {key: state[key] for key in (
        "frame", "eef_frame", "ee_pose", "commanded_ee_pose",
        "right_closed_ratio_command", "gripper_fraction", "fault",
        "motion_enabled") if key in state}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--max-steps", type=int, default=60)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--view-dir", type=Path, required=True)
    parser.add_argument("--external-camera", default="/dev/video0")
    parser.add_argument("--external-fps", type=float, default=5.0)
    parser.add_argument("--no-external-recording", action="store_true")
    parser.add_argument("--resume-grasped", action="store_true")
    parser.add_argument("--start-step", type=int, default=0)
    parser.add_argument("--operator-guidance", default="",
                        help="Current operator observation/intent; verify against fresh images")
    parser.add_argument("--approach-image-edge", type=int, default=960,
                        help="Max image edge during approach; contact views stay full resolution")
    args = parser.parse_args()
    if args.approach_image_edge < 640:
        parser.error("--approach-image-edge must be at least 640")
    ui = dict(mode="rollout", task="蛋糕 → 蓝色盘子", logs=[], stage="approach",
              max_steps=args.max_steps, inference_s=0.0, inference_total_s=0.0)
    try:
        old_ui = json.loads((args.view_dir / "status.json").read_text())
        for key in ("planned_delta_pose", "delta_status", "delta_iteration"):
            if key in old_ui:
                ui[key] = old_ui[key]
    except (OSError, ValueError):
        pass
    def show(**fields):
        ui.update(fields)
        args.view_dir.mkdir(parents=True, exist_ok=True)
        temp = args.view_dir / "sdk_status.tmp"
        temp.write_text(json.dumps(ui, ensure_ascii=False))
        temp.replace(args.view_dir / "status.json")

    robot = R1A7PickPlaceSDK(args.session)
    reference = load_reference(ROOT / "data/r1_a7_task/2026_09_22_robot_r1a7_dex1")
    ledger = ControlLedger("pick yellow cake and place inside blue plate", args.max_steps)
    ledger.step = args.start_step
    trace = EpisodeTrace(args.session)
    if trace.trace.exists():
        for line in trace.trace.read_text().splitlines()[-4:]:
            row = json.loads(line)
            old = row["proposal"]
            ledger.recent.append(dict(action=old["action"], stage=old["stage"],
                                      observed=old["observation"], result=row["execution"]["status"],
                                      historical=True))
    recording_dir = trace.root / time.strftime("recording_%Y%m%d_%H%M%S")
    planner = AstraClient(
        "gpt-6-astra", "https://ys3.chunxueai.com/v1", "UNITREE_OPENAI_API_KEY",
        timeout=25, proxy="http://127.0.0.1:7897", instructions=PROMPT,
        schema=ACTION_SCHEMA, schema_name="r1a7_sdk_action")
    recorder = None if args.no_external_recording else ExternalVideoRecorder(
        recording_dir, device=args.external_camera, fps=args.external_fps)
    try:
        show(phase="准备录像与观测", planning_phase="observing", iteration=args.start_step)
        if recorder is not None:
            path = recorder.start()
            print(f"external recording: {path} ({args.external_fps:g} fps, not policy input)", flush=True)
        observation = robot.observe()
        if observation.state.get("fault"):
            raise SDKError(observation.state["fault"])
        if not args.dry_run and not observation.state.get("motion_enabled"):
            raise SDKError("real rollout requires motion-enabled server")
        if args.dry_run and observation.state.get("motion_enabled"):
            raise SDKError("--dry-run requires a read-only server")
        if args.resume_grasped:
            if observation.state["right_closed_ratio_command"] <= 0.5:
                raise SDKError("--resume-grasped requires a closed right gripper")
            robot._trial_taken = True
            robot._grasp_verified = True
            ledger.step = args.start_step
            ledger.phase = "transfer"
            ledger.held_object = "cake"
            ledger.grasp_verified = True
            ledger.first_unmet_gate = "transfer over plate, then verify support"
        previous = []
        inference_total_s = 0.0
        for step in range(args.start_step + 1, args.max_steps + 1):
            if (args.view_dir / "stop_requested").exists():
                raise SDKError("operator STOP")
            images = []
            for alias, camera in (("top", "head_right"), ("wrist", "wrist_right")):
                frame = cv2.imread(observation.images[alias])
                if frame is None:
                    raise SDKError(f"cannot read {alias} observation")
                images.append((camera, frame))
            show(iteration=step, phase="闭环执行", planning_phase="inference",
                 planning_started_at=time.time(),
                 right_gripper_ratio=observation.state["right_closed_ratio_command"])
            planning_started = time.monotonic()
            request_images = planner_views(images, ledger.phase, args.approach_image_edge)
            if robot.task_state["trial_taken"] and not robot.task_state["grasp_verified"]:
                request_images = previous + images
            context = {"ledger": ledger.context(), "lift_state": robot.lift_state, "robot_state": planner_state(observation.state),
                       "operator_guidance": args.operator_guidance,
                       "teleop_reference": reference}
            try:
                proposal = planner.decide(
                    context, request_images)
            except Halt as exc:
                inference_elapsed = time.monotonic() - planning_started
                inference_total_s += inference_elapsed
                if "TimeoutError" not in str(exc):
                    raise
                print("Planning timeout: discard capture, hold and re-observe", flush=True)
                show(phase="推理超时 · 保持并重新观察", planning_phase="observing",
                     inference_s=inference_elapsed, inference_total_s=inference_total_s)
                observation = robot.observe()
                continue
            inference_elapsed = time.monotonic() - planning_started
            inference_total_s += inference_elapsed
            show(planning_phase="checking", inference_s=inference_elapsed,
                 inference_total_s=inference_total_s,
                 observation_text=proposal.get("observation", ""),
                 action_prompt=proposal.get("reason", ""), stage=proposal.get("stage", "approach"))
            with (trace.root / "planning_metrics.jsonl").open("a") as stream:
                stream.write(json.dumps(dict(step=step, inference_s=inference_elapsed,
                    context_chars=len(json.dumps(context)),
                    images=[dict(camera=name, width=frame.shape[1], height=frame.shape[0])
                            for name, frame in request_images])) + "\n")
            action = Action.from_dict(proposal)
            if action.action == "move":
                show(planned_delta_pose=dict(
                    translation_m=action.dpos_m,
                    rotvec_rad=Rotation.from_euler("xyz", action.drot_deg, degrees=True).as_rotvec().tolist()),
                    delta_status="待下发", delta_iteration=step)
            if (args.view_dir / "stop_requested").exists():
                raise SDKError("operator STOP")
            show(planning_phase="executing")
            started = time.monotonic()
            try:
                result = robot.step(action)
            except Exception:
                if action.action == "move":
                    show(delta_status="执行未确认：请查看保持原因")
                raise
            if action.action == "move":
                show(delta_status="已下发：" + result.robot_result.get("timing", {}).get("outcome", result.status))
            timing = result.robot_result.get("timing", {})
            show(planning_phase="done", execution_s=time.monotonic()-started,
                 effective_hz=timing.get("effective_hz", ui.get("effective_hz", 0)),
                 max_gap_ms=timing.get("max_gap_ms", ui.get("max_gap_ms", 0)),
                 right_gripper_ratio=result.robot_result.get("after", {}).get(
                     "right_closed_ratio_command", observation.state["right_closed_ratio_command"]))
            ledger.update(action, result, robot.task_state)
            trace.append(step=ledger.step, before=observation, action=action,
                         result=result, ledger=ledger,
                         elapsed_s=time.monotonic() - started)
            print(f"step={ledger.step} action={action.action} gate={ledger.first_unmet_gate}", flush=True)
            if result.status == "done":
                show(phase="放置成功 · 已视觉确认")
                return 0
            if result.status == "stopped":
                raise SDKError(result.message or "agent requested stop")
            if args.dry_run:
                print("DRY RUN PASS", flush=True)
                return 0
            if result.observation is None:
                raise SDKError("state-changing action returned no fresh observation")
            observation = result.observation
            previous = [("before_" + name, frame) for name, frame in images]
        raise SDKError("step budget exhausted")
    except (Exception, KeyboardInterrupt) as exc:
        show(phase="保持 / HOLD", planning_phase="hold",
             reason=str(exc), action_prompt=str(exc))
        print(f"HOLD {exc}", flush=True)
        return 2
    finally:
        if recorder is not None:
            try:
                recorder.stop()
            except SDKError as exc:
                print(f"RECORDING ERROR {exc}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
