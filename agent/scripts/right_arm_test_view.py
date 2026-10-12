#!/usr/bin/env python3
"""Read-only two-camera rollout dashboard in the existing Qt5-backed window."""
import argparse,json,sys,time
from collections import deque
from pathlib import Path
REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
import cv2
import numpy as np
from teleop.image_client import FourCameraSubscriber
from agent.visual_freshness import VisualFreshness
from agent.rollout_display import render_dashboard
from agent.scripts.astra_grasp_rollout import ReadOnlyDDS

def main():
    p=argparse.ArgumentParser();p.add_argument('log_dir',type=Path);args=p.parse_args()
    args.log_dir.mkdir(parents=True, exist_ok=True)
    sub=FourCameraSubscriber('192.168.123.164',5555);probe=ReadOnlyDDS('enp5s0');freshness=VisualFreshness()
    window='R1-A7 | Cake to Plate | SPACE: Hold'
    sub.start();cv2.namedWindow(window,cv2.WINDOW_NORMAL);cv2.resizeWindow(window,1440,864)
    ui_started = time.monotonic()
    def click(event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN and 1300<=x<=1560 and 768<=y<=815:
            (args.log_dir/'stop_requested').touch()
    cv2.setMouseCallback(window,click)
    saved=0;lowstate_samples=deque();last_lowstate_stamp=None
    try:
        while True:
            try:state=json.loads((args.log_dir/'status.json').read_text())
            except (OSError,ValueError):state={}
            with probe.lock:
                hand=probe.messages.get('right_state')
                ratio=float(np.clip(1-hand.states[0].q/5,0,1)) if hand is not None else float('nan')
                lowstate_stamp=probe.times.get('lowstate',-float('inf'))
                dds_age=time.monotonic()-lowstate_stamp
            if lowstate_stamp!=last_lowstate_stamp and np.isfinite(lowstate_stamp):
                lowstate_samples.append(lowstate_stamp);last_lowstate_stamp=lowstate_stamp
            cutoff=time.monotonic()-1.0
            while lowstate_samples and lowstate_samples[0]<cutoff:lowstate_samples.popleft()
            control_hz=(len(lowstate_samples)-1)/(lowstate_samples[-1]-lowstate_samples[0]) if len(lowstate_samples)>1 else 0.0
            frames,fid,stamp=sub.get()
            ages=freshness.update(frames,fid) if frames is not None else {}
            canvas=render_dashboard(state,frames,dict(ui_elapsed_s=time.monotonic()-ui_started,hand_ratio=ratio,dds_age=dds_age,control_hz=control_hz,camera_age=(time.time_ns()-stamp)/1e9 if stamp else 999),ages)
            cv2.imshow(window,canvas);(args.log_dir/'view_alive').touch()
            if time.monotonic()-saved>2:
                cv2.imwrite(str(args.log_dir/'dashboard.jpg'),canvas);saved=time.monotonic()
            key=cv2.waitKey(20)&0xff
            visible=cv2.getWindowProperty(window,cv2.WND_PROP_VISIBLE)>=1
            if key in (32,ord('s'),ord('q')) or not visible:
                (args.log_dir/'stop_requested').touch()
                if key==ord('q') or not visible:break
    finally:
        (args.log_dir/'stop_requested').touch();sub.close();probe.close();cv2.destroyAllWindows()
if __name__=='__main__':main()
