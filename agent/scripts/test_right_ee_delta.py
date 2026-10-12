#!/usr/bin/env python3
"""Interactive, observed 10-mm vertical EE steps; no queued trajectory.

Waist yaw is an internal coordinate gauge here: a pure +Z increment with zero
rotation is invariant to the fixed waist yaw. This does NOT estimate its angle
or publish waist commands. Commands to inactive joints are handed over verbatim.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time

REPO_ROOT = Path(__file__).resolve().parents[2]
ROOT = REPO_ROOT
sys.path.insert(0,str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation as R
from agent.scripts.astra_grasp_rollout import ReadOnlyDDS, Journal, fresh_frames
from agent.astra_policy import Decision, Halt, selected_views
from teleop.image_client import FourCameraSubscriber
from agent.astra_robot import RightOnlyController
from robot.config import R1A7Config, RIGHT_ARM_JOINTS
from robot.arm_control import RealArmController


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--takeover-pid',type=int,required=True)
    ap.add_argument('--view-dir',type=Path,required=True)
    args=ap.parse_args()
    log=Journal(ROOT/'logs'/('right_ee_delta_'+time.strftime('%Y%m%d_%H%M%S')))
    probe=ReadOnlyDDS('enp5s0')
    sub=FourCameraSubscriber('192.168.123.164',5555); sub.start()
    c=None
    state={'phase':'EE PRECHECK','reason':'Vertical EE test; one observed step at a time. No grasp.'}
    def show(**values):
        state.update(values)
        tmp=args.view_dir/'status.tmp'
        tmp.write_text(json.dumps(state)); tmp.replace(args.view_dir/'status.json')
    def sensors():
        probe.require_fresh()
        if (args.view_dir/'stop_requested').exists(): raise Halt('STOP requested in viewer')
        if time.time()-(args.view_dir/'view_alive').stat().st_mtime>1: raise Halt('viewer unresponsive')
        with sub.lock:
            if time.time_ns()-sub.received_at_ns>500_000_000:raise Halt('camera stale')
    try:
        frames,fid,_=fresh_frames(sub,0,time.time_ns(),timeout=5)
        time.sleep(.5);sensors()
        with probe.lock:
            cmd=probe.messages.get('lowcmd')
            if cmd is None:raise Halt('no current controller commands to hand over')
            joints={n:float(cmd.motor_cmd[i].q) for n,i,_,_ in probe.real_joints}
            left=float(probe.messages['left_state'].states[0].q)
        joints['waist_yaw_joint']=0.0  # +Z-only coordinate gauge; never sent
        snapshot={'joint_commands':joints,'left_closed_ratio':float(np.clip(1-left/5,0,1))}
        c=RightOnlyController(R1A7Config(reset_blend_frames=100),snapshot,[-.2,.8,-.85,.05,-.45,.7])
        # Soft per-target joint-delta Halt is disabled; low-level 0.04 rad/20ms
        # still rate-limits published commands.
        measured=probe.arm_head_positions()
        c.q[c.right_arm_q_indices]=measured[7:14]
        initial_pose,initial_quat=c.pose()
        c.home_right=measured[7:14].copy()  # recovery to test entry, never default home
        log.write('entry',right_measured=measured[7:14].tolist(),ee_position=initial_pose.tolist(),commands=joints)
        first=c.solve_step(Decision('move',(0,0,.01),(0,0,0),'operator-requested vertical control test'))
        print('First +10mm IK validated; max joint target delta:',float(np.max(np.abs(first-c.q))),flush=True)
        # Only replace the known test process; do not terminate an arbitrary PID.
        process_cmd=Path(f'/proc/{args.takeover_pid}/cmdline').read_bytes()
        if b'scripts/test_right_arm_motion.py' not in process_cmd:
            raise Halt('takeover PID is not the previous wrist-test controller')
        sensors()
        old=probe.arm_head_positions()
        if np.max(np.abs(old-measured))>.01:raise Halt('robot moved during precheck')
        os.kill(args.takeover_pid,signal.SIGTERM)
        deadline=time.monotonic()+2
        while Path(f'/proc/{args.takeover_pid}').exists():
            if time.monotonic()>deadline:raise Halt('previous writer did not exit')
            time.sleep(.01)
        c.arm_ctrl=RealArmController(c.config,'enp5s0',initial_command_q=c.initial_command,skip_motion_switcher=True)
        c.motion_mode=True;c.ready=True
        fid=0;step=0
        def watch():
            sensors()
            actual=probe.arm_head_positions()
            if np.max(np.abs(actual[:7]-c.initial_command[:7]))>.03:raise Halt('left arm drift')
            if np.max(np.abs(actual[14:]-c.initial_command[14:]))>.03:raise Halt('head drift')
            p,r=c.measured_pose()
            delta=(p-initial_pose)*1000
            show(ee_delta_mm=delta.tolist(),measured=float(actual[12]))
            if np.linalg.norm(delta[:2])>20 or delta[2]<-5 or delta[2]>110:
                raise Halt('EE left test corridor')
            return p,r,actual
        show(phase='EE READY',start=float(measured[12]),target=float(c.initial_command[12]),ee_delta_mm=[0,0,0])
        print('READY. step = one +Z step <=10mm; return = 100-frame recovery. No autonomous sequence.',flush=True)
        while True:
            try:
                action=input('ee> ').strip()
                if action in ('hand 0','hand 1'):
                    sensors()
                    from robot.eef.dex1 import Dex1
                    if c.hand_ctrl is None:
                        c.hand_ctrl=Dex1(initial_ratios=(c.left_ratio,0),publish_left=False)
                    ratio=float(action[-1])
                    c.hand_ctrl.set_gripper_ratios(c.left_ratio,ratio)
                    show(phase='RIGHT GRIPPER TEST',right_gripper_ratio=ratio)
                    deadline=time.monotonic()+3
                    while time.monotonic()<deadline:
                        sensors()
                        with probe.lock:position=float(probe.messages['right_state'].states[0].q)
                        if abs(position-(1-ratio)*5)<.2:break
                        time.sleep(.02)
                    else:raise Halt('right gripper did not reach requested 0/1 position')
                    frames,fid,_=fresh_frames(sub,fid,time.time_ns(),watchdog=sensors)
                    log.images(100+step,selected_views(frames,'head')+selected_views(frames,'wrist'))
                    log.write('hand_test',ratio=ratio,measured_rad=position)
                    print('RIGHT HAND',ratio,'measured rad',position,flush=True)
                    continue
                if action=='return':
                    sensors();c.return_home(sensors);log.write('returned');show(phase='EE RETURNED');break
                if action!='step':continue
                sensors();c.refresh();p0,r0=c.pose()
                remaining=.1-(p0[2]-initial_pose[2])
                if remaining<=.002:
                    show(phase='EE 100mm REACHED'); print('100mm test range reached; holding.',flush=True);continue
                dz=min(.01,remaining)
                q=c.solve_step(Decision('move',(0,0,dz),(0,0,0),'single operator-reviewed step'))
                pgoal,rgoal=c.pose(q)
                log.write('target',step=step,delta_z=dz,ee_target=pgoal.tolist(),right_q=q[c.right_arm_q_indices].tolist())
                show(phase=f'EE STEP {step+1}',ee_target_delta_mm=((pgoal-initial_pose)*1000).tolist(),target=float(q[c.right_arm_q_indices[5]]))
                sensors();c.q=q.copy();c._output_command()
                stable=0;deadline=time.monotonic()+4
                while time.monotonic()<deadline:
                    p,r,a=watch()
                    error=float(np.linalg.norm(p-pgoal))
                    rot=float((R.from_quat(r).inv()*R.from_quat(rgoal)).magnitude())
                    log.write('feedback',step=step,ee_position=p.tolist(),position_error_m=error,rotation_error_rad=rot)
                    # This is a gross motion test, not precision pre-grasp.
                    stable=stable+1 if error<=.005 and rot<=.02 else 0
                    if stable>=10:break
                    time.sleep(.02)
                else:raise Halt('EE tracking failed: >5mm position or >0.02rad orientation error')
                frames,fid,_=fresh_frames(sub,fid,time.time_ns(),watchdog=sensors)
                step+=1
                log.images(step,selected_views(frames,'head')+selected_views(frames,'wrist'))
                p,r,a=watch()
                show(phase='EE OBSERVE / HOLD',reason=f'Step {step} done; waiting for review of NEW images before next step.')
                print(json.dumps({'step':step,'measured_delta_mm':((p-initial_pose)*1000).tolist(),'images':str(log.path/f'{step:04d}_head_left.jpg')}),flush=True)
            except (Exception,KeyboardInterrupt) as exc:
                c.freeze();show(phase='HOLD',reason=str(exc) or 'interrupted')
                log.write('halt',reason=str(exc));print('HALT; no further steps:',exc,flush=True)
                # Latch failure: only return is accepted after an explicit review.
                while True:
                    try:
                        if input('halt> (return only) ').strip()!='return':continue
                        sensors();c.return_home(sensors);show(phase='EE RETURNED');return
                    except (Exception,KeyboardInterrupt) as recovery:
                        c.freeze();print('Holding:',recovery,flush=True)
    finally:
        if c is not None:c._on_stop()
        probe.close();sub.close();print('Log:',log.path,flush=True)

if __name__=='__main__':main()
