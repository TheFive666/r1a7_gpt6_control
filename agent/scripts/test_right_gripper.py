#!/usr/bin/env python3
"""Interactive right-only Dex1 0/1 diagnostic; no arm publisher."""
from pathlib import Path
import json
import sys
import time

REPO_ROOT = Path(__file__).resolve().parents[2]
ROOT = REPO_ROOT
sys.path.insert(0,str(ROOT))
from agent.scripts.astra_grasp_rollout import ReadOnlyDDS, Journal, fresh_frames
from teleop.image_client import FourCameraSubscriber
from agent.astra_policy import selected_views, Halt

p=ReadOnlyDDS('enp5s0');s=FourCameraSubscriber('192.168.123.164',5555);s.start()
hand=None
log=Journal(ROOT/'logs'/('right_hand_test_'+time.strftime('%Y%m%d_%H%M%S')))
view=ROOT/'logs/right_arm_test_20260921_165123'
try:
 frames,fid,_=fresh_frames(s,0,time.time_ns(),timeout=4)
 time.sleep(.4);p.require_fresh()
 with p.lock:
  if time.monotonic()-p.times.get('right_cmd',-1e9)<.75:raise Halt('another right-hand writer is active')
  left=max(0,min(1,1-float(p.messages['left_state'].states[0].q)/5))
  right=max(0,min(1,1-float(p.messages['right_state'].states[0].q)/5))
 from robot.eef.dex1 import Dex1
 hand=Dex1(initial_ratios=(left,right),publish_left=False)
 print('READY: enter 0=open, 1=close, exit=stop hand writer. Arm remains held.',flush=True)
 counter=0
 while True:
  cmd=input('hand> ').strip()
  if cmd=='exit':break
  if cmd not in ('0','1'):continue
  p.require_fresh()
  if (view/'stop_requested').exists():raise Halt('viewer STOP requested')
  ratio=float(cmd);hand.set_gripper_ratios(left,ratio)
  state=json.loads((view/'status.json').read_text());state.update(right_gripper_ratio=ratio,phase='ARM HOLD / HAND TEST',reason='Right hand 0/1 test. Arm remains held after tracking failure.')
  (view/'status.json').write_text(json.dumps(state))
  deadline=time.monotonic()+4;stable=0
  while time.monotonic()<deadline:
   p.require_fresh()
   if (view/'stop_requested').exists():raise Halt('viewer STOP requested')
   with s.lock:
    if time.time_ns()-s.received_at_ns>500_000_000:raise Halt('camera stale')
   with p.lock:measured=float(p.messages['right_state'].states[0].q)
   stable=stable+1 if abs(measured-(1-ratio)*5)<.2 else 0
   if stable>=10:break
   time.sleep(.02)
  else:raise Halt('right gripper did not reach target')
  frames,fid,_=fresh_frames(s,fid,time.time_ns(),watchdog=p.require_fresh)
  counter+=1;log.images(counter,selected_views(frames,'wrist'))
  log.write('verified',closed_ratio=ratio,measured_motor_rad=measured)
  print('VERIFIED',ratio,'measured motor rad',measured,'image',log.path/f'{counter:04d}_wrist_right.jpg',flush=True)
except (Exception,KeyboardInterrupt) as exc:
 print('HALT:',exc,flush=True)
 if hand is not None:hand.set_gripper_ratios(left,0)
finally:
 if hand is not None:hand.stop()
 p.close();s.close();print('Log:',log.path,flush=True)
