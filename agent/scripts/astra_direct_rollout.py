#!/usr/bin/env python3
"""Direct EEF phase-one entry. Dry-run never opens a robot command writer.

A single fresh proposal is checked in dry-run; no fictitious action history.
Holds and explicitly resumes within one process, keeping the DDS writer alive.
"""
from pathlib import Path
import sys
import time
import json
import argparse
import os
import signal
from concurrent.futures import ThreadPoolExecutor
REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
import numpy as np
from agent.scripts.astra_grasp_rollout import ReadOnlyDDS,Journal,fresh_frames,hold_until_home
from teleop.image_client import FourCameraSubscriber
from agent.astra_direct_policy import direct_client,direct_views,DirectSafety,DirectPhase,pose_dict
from agent.astra_robot import RightOnlyController
from robot.config import R1A7Config
from agent.astra_policy import Halt
from agent.cake_detector import detect_cake, annotate
from agent.astra_grasp_stage import close_and_verify
from agent.visual_freshness import VisualFreshness


def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--net',default='enp5s0')
 p.add_argument('--target',default='桌上的黄色蛋糕；第一阶段到稳定 pre-grasp，保持右手打开。')
 p.add_argument('--view-dir',type=Path,default=Path('logs/right_arm_test_20260921_165123'))
 p.add_argument('--motion-mode',action='store_true')
 p.add_argument('--enable-dex1',action='store_true')
 p.add_argument('--takeover-pid',type=int)
 p.add_argument('--prepared-handover-pid',type=int)
 p.add_argument('--grasp',action='store_true')
 p.add_argument('--high-lift-m',type=float,default=.20)
 p.add_argument('--max-steps',type=int,default=60)
 args=p.parse_args()
 if args.motion_mode and not args.enable_dex1:p.error('--motion-mode requires --enable-dex1')
 if not 0<=args.high_lift_m<=.30:p.error('--high-lift-m must be between 0 and 0.30 m')
 if args.max_steps<1:p.error('--max-steps must be positive')
 if args.takeover_pid is not None:
  p.error('Process takeover disabled: stopping the current writer can trigger damping. Resume the existing persistent controller instead.')
 log=Journal(Path('logs')/('astra_direct_'+time.strftime('%Y%m%d_%H%M%S')))
 metrics={'iteration':0,'logs':[],'inference_s':0.,'inference_total_s':0.,'execution_s':0.,'step_s':0.,'started_at':time.time()}
 sub=FourCameraSubscriber('192.168.123.164',5555);probe=ReadOnlyDDS(args.net);c=None
 def show(phase,reason):
  if args.view_dir.exists():
   state={'mode':'rollout','phase':phase,'reason':reason,'start':0,'target':0,'right_gripper_ratio':getattr(c,'right_ratio',0.0),**metrics}
   if c is not None:
    state['ee_position_mm']=(c.measured_pose()[0]*1000).tolist() if c.ready else (c.pose()[0]*1000).tolist()
   state['logs']=(metrics['logs']+[phase+': '+reason[:110]])[-5:]
   tmp=args.view_dir/'direct.tmp';tmp.write_text(json.dumps(state));tmp.replace(args.view_dir/'status.json')
 try:
  sub.start();fresh_frames(sub,0,time.time_ns(),timeout=5)
  deadline=time.monotonic()+4
  while True:
   time.sleep(.05)
   try:probe.require_fresh();break
   except Halt:
    if time.monotonic()>deadline:raise
  with probe.lock:
   cmd=probe.messages.get('lowcmd');state=probe.messages['lowstate']
   # Following operator recovery from damping, seed a fresh stationary hold
   # when no previous writer remains. Never restore stale pre-damping targets.
   if cmd is None:
    joints={n:float(state.motor_state[i].q) for n,i,_,_ in probe.real_joints}
   else:
    joints={n:float(cmd.motor_cmd[i].q) for n,i,_,_ in probe.real_joints}
   joints['waist_yaw_joint']=float(state.motor_state[13].q)
   left=float(probe.messages['left_state'].states[0].q)
   right_command=probe.messages.get('right_cmd')
   right=float(right_command.cmds[0].q) if right_command is not None else float(probe.messages['right_state'].states[0].q)
  snapshot={'joint_commands':joints,'left_closed_ratio':float(np.clip(1-left/5,0,1)),
            'right_closed_ratio':float(np.clip(1-right/5,0,1))}
  workspace=[-.2,.8,-.85,.05,-.45,.7]
  c=RightOnlyController(R1A7Config(),snapshot,workspace)
  log.write('limits',target_control_hz=30.0,interpolation='right joint linear within one validated EEF action',head_eef_step_m=.020,wrist_eef_step_m=.020,near_grasp_eef_step_m=.003,joint_action_soft_limit='disabled',lowlevel_joint_delta_rad=.04,lowlevel_dt_s=.02)
  time.sleep(.2)
  phase=DirectPhase();history=[];fid=0;after=time.time_ns()
  pool=ThreadPoolExecutor(max_workers=1)
  visual_freshness=VisualFreshness()
  def watch():
   probe.require_fresh()
   if args.motion_mode:
    if (args.view_dir/'stop_requested').exists():raise Halt('visualization STOP requested')
    if time.time()-(args.view_dir/'view_alive').stat().st_mtime>1:raise Halt('visualization stale')
   with sub.lock:
    if time.time_ns()-sub.received_at_ns>500_000_000:raise Halt('camera stale')
   live,live_id,_=sub.get()
   if live is not None:
    metrics['image_unchanged_s']=visual_freshness.require_live(live,live_id)
   if c.ready:
    actual_now=probe.arm_head_positions()
    if np.max(np.abs(actual_now[:7]-c.initial_command[:7]))>.04:raise Halt('left arm drift')
    if np.max(np.abs(actual_now[14:]-c.initial_command[14:]))>.04:raise Halt('head drift')
    if not c.arm_ctrl.real.writer_thread.is_alive():raise Halt('DDS writer stopped')
  def infer_checked(client, observation, views):
   captured=time.time_ns();actual=probe.arm_head_positions().copy()
   future=pool.submit(client.decide,observation,views)
   while not future.done():
    watch()
    if time.time_ns()-captured>30e9:raise Halt('grasp decision expired')
    time.sleep(.02)
   result=future.result();watch()
   if time.time_ns()-captured>30e9:raise Halt('grasp decision expired')
   if np.max(np.abs(probe.arm_head_positions()-actual))>.015:raise Halt('robot moved during grasp inference')
   latest,_,_=sub.get()
   for (_,old),(_,new) in zip(views[-2:],direct_views(latest)):
    if old.shape!=new.shape or np.mean(np.abs(old.astype(float)-new.astype(float)))/255>.06:
     raise Halt('scene changed during grasp inference')
   return result
  def observe_grasp():
   nonlocal fid
   frames,fid,_=fresh_frames(sub,fid,time.time_ns(),watchdog=watch)
   log.images(100000+fid,direct_views(frames))
   return frames,fid
  observe_grasp.watch=watch
  def run_rollout():
   nonlocal fid,after
   if c.right_ratio>.05:
    raise Halt('Right hand already holding closed; retain grip, do not restart open-hand approach.')
   for iteration in range(args.max_steps):
    step_started=time.monotonic();metrics["iteration"]+=1
    frames,fid,received=fresh_frames(sub,fid,after,watchdog=watch)
    actual=probe.arm_head_positions()
    measured=c.q.copy()
    measured[c.left_arm_q_indices]=actual[:7];measured[c.right_arm_q_indices]=actual[7:14]
    cp,cq=c.pose();mp,mq=c.pose(measured)
    obs={'task':args.target,'primary_view':phase.primary_view,'frame_id':fid,
         'current_right_eef':pose_dict(cp,cq),'current_right_eef_source':'persistent commanded FK',
         'measured_right_eef':pose_dict(mp,mq),'right_arm_joint_state':actual[7:14].tolist(),
         'right_arm_joint_command':c.q[c.right_arm_q_indices].tolist(),
         'tool_geometry':c.tool_geometry,'recent_executed_history':history[-5:],
         'phase':'pregrasp_only','workspace':c.workspace.tolist(),
         'camera_unchanged_seconds':metrics.get('image_unchanged_s',{}),
         'motion_limits_m':{'clear_approach':.020,'near_grasp':.003}}
    views=direct_views(frames)
    detections={name:detect_cake(frame,tabletop_only=name.startswith('head')) for name,frame in views}
    obs['target_candidates_pixels']=detections
    obs['detector_note']='Color contours only, not semantic identity or depth. Head ROI bottom 45 percent for fixed head. Confirm cake identity from RGB.'
    if not any(detections.values()):raise Halt('yellow cake candidate lost')
    log.images(iteration,[(name,annotate(frame,detections[name])) for name,frame in views]);log.write('observation',observation=obs)
    show('DIRECT EEF / '+phase.primary_view.upper(),'Reading head-right and wrist-right fresh images; waiting for ONE target.')
    future=pool.submit(direct_client(timeout=30).decide,obs,views)
    while not future.done():
     watch()
     if time.time_ns()-received>30e9:raise Halt('decision expired')
     time.sleep(.02)
    raw=future.result()
    metrics["inference_s"]=(time.time_ns()-received)/1e9
    metrics["inference_total_s"]+=metrics["inference_s"]
    log.write('astra_output',output=raw);print('ASTRA',json.dumps(raw,ensure_ascii=False),flush=True)
    watch()
    if time.time_ns()-received>30e9:raise Halt('decision expired')
    latest,_,_=sub.get()
    for (_,old),(_,new) in zip(views,direct_views(latest)):
     if old.shape!=new.shape or np.mean(np.abs(old.astype(float)-new.astype(float)))/255>.06:raise Halt('scene changed during inference')
    if np.max(np.abs(probe.arm_head_positions()-actual))>.015:raise Halt('robot moved during inference')
    safe=DirectSafety(workspace).clamp(raw,cp,cq,phase.primary_view);log.write('safety',target=safe.as_dict())
    metrics['eef_target']=safe.as_dict()['right_eef_target']
    metrics['delta_mm']=((safe.position-cp)*1000).tolist()
    metrics['near_grasp']=raw['near_grasp']
    metrics['cake_between_fingers']=raw['cake_between_fingers']
    metrics['tips_at_grasp_height']=raw['tips_at_grasp_height']
    if safe.status=='move':
     q=c.solve_step(safe.as_delta(cp,cq))
     c.collisions.check(measured,q)
     log.write('ik_validated',right_q=q[c.right_arm_q_indices].tolist(),inactive_unchanged=bool(np.array_equal(q[c.fixed_indices],c.q[c.fixed_indices])))
    if not args.motion_mode:
     show('DIRECT DRY RUN PASS','Proposal and safety validated; no motion executed.')
     print('DRY_RUN_PASS',log.path,flush=True);return 0
    if not c.ready:
     c.attach_hardware(args.net,probe,held_pid=args.prepared_handover_pid)
     deadline=time.monotonic()+4
     while True:
      watch()
      with probe.lock:opened=abs(float(probe.messages['right_state'].states[0].q)-5)<.15
      if opened:break
      if time.monotonic()>deadline:raise Halt('right hand not open')
      time.sleep(.02)
     # Opening/takeover is itself an action. Discard this pre-takeover
     # proposal; request fresh images before ANY arm movement.
     after=time.time_ns();show('DIRECT READY','Handover complete; new observation required.');continue
    if safe.status=='move':
     watch()
     if time.time_ns()-received>30e9:raise Halt('proposal expired during IK')
     show('DIRECT MOVE',safe.reason)
     execution_started=time.monotonic()
     try:
      c.execute_one(q,watch,timeout=5)
     finally:
      metrics["execution_s"]=time.monotonic()-execution_started
      metrics["step_s"]=time.monotonic()-step_started
      metrics.update(getattr(c,'last_execution',{}))
      metrics['measured_delta_mm']=((c.measured_pose()[0]-mp)*1000).tolist()
      log.write('execution_metrics',metrics=metrics)
      show('EXECUTION FEEDBACK','Measured pose and timing; outcome checked separately.')
     p_after,r_after=c.measured_pose()
     record={'frame_id':fid,'executed_target':safe.as_dict(),'measured_eef_after':pose_dict(p_after,r_after),
             'measured_delta_m':(p_after-mp).tolist(),
             'commanded_delta_m':(safe.position-cp).tolist(), 'target_candidates_before':detections}
     history.append(record);log.write('action_settled',action=record)
     metrics['measured_delta_mm']=((p_after-mp)*1000).tolist()
     print('EXECUTED',json.dumps(record),flush=True)
    metrics['step_s']=time.monotonic()-step_started
    metrics['logs']=(metrics['logs']+[f"step {metrics['iteration']} {safe.status} total={metrics['step_s']:.2f}s delta_mm={metrics.get('delta_mm')}"])[-5:]
    log.write('step_metrics',metrics=metrics)
    show('STEP COMPLETE',safe.reason)
    after=time.time_ns()
    if phase.accept(safe,fid):
     show('PREGRASP STABLE','Three fresh confirmations; right hand remains open.')
     log.write('pregrasp_stable')
     if args.grasp:close_and_verify(c,probe,observe_grasp,infer_checked,show,log,high_lift_m=args.high_lift_m)
     return 0
   raise Halt('step budget exhausted')
  try:
   while True:
    try:
     result=run_rollout()
     return result
    except (Exception,KeyboardInterrupt) as exc:
     if not c.ready:raise
     c.freeze()
     log.write('hold',reason=str(exc),pid=os.getpid())
     show('HOLD / DIRECT STOP',str(exc))
     print('HOLD',str(exc),'writer remains alive; explicit same-process resume required',flush=True)
     resume_file=args.view_dir/'resume_rollout.json'
     hold_started=time.time_ns()
     while True:
      # No RPC or mode change; no automatic retry. A new operator request
      # must name this exact PID and be newer than this latched stop.
      try:
       request=json.loads(resume_file.read_text())
       if request.get('pid')==os.getpid() and request.get('requested_at_ns',0)>hold_started:
        resume_file.unlink()
        probe.require_fresh()
        if not c.arm_ctrl.real.writer_thread.is_alive():raise Halt('DDS writer stopped; cannot resume')
        phase=DirectPhase();fid=0;after=time.time_ns()
        break
      except (OSError,ValueError):pass
      time.sleep(.1)
  finally:pool.shutdown(wait=False,cancel_futures=True)
 except (Exception,KeyboardInterrupt) as e:
  log.write('halt',reason=str(e));show('HOLD / DIRECT STOP',str(e));print('HALT',str(e),flush=True)
  return 2
 finally:
  if c is not None:
   if c.ready:
    c.freeze()
    hold_until_home(c,probe,log)
   c._on_stop()
  sub.close();probe.close();print('Log:',log.path,flush=True)
 return 0

if __name__=='__main__':raise SystemExit(main())
