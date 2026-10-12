"""Fail-closed read-only mode watchdog; never changes robot operating mode.

Empty CheckMode.name means released high-level ownership, NOT a documented
R1 debugging FSM ID. The operator must establish debugging mode. This watchdog
checks that observable ownership and lowstate mode fields remain unchanged.
"""
import threading
import time
from agent.astra_policy import Halt


class DebugModeGuard:
    def __init__(self, probe, on_fault=None):
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        self.probe=probe
        self.on_fault=on_fault
        self.client=MotionSwitcherClient();self.client.SetTimeout(.5);self.client.Init()
        self.stop_event=threading.Event();self.failure=None;self.last_ok=0.
        self.release_required=False
        self.thread=None
        probe.require_fresh()
        with probe.lock:
            state=probe.messages['lowstate']
            self.baseline=(int(state.mode_machine),int(state.mode_pr))
        self._query()

    def _query(self):
        code,result=self.client.CheckMode()
        if code!=0 or not isinstance(result,dict):
            # A failed query is not a mode transition. Latch motion STOP,
            # but keep the holding writer alive rather than inducing damping.
            raise Halt(f'mode query failed (code={code}); stop motion, retain hold')
        if result.get('name')!='':
            self.release_required=True
            raise Halt('another motion owner detected; release lowcmd writer')
        self.probe.require_fresh()
        with self.probe.lock:
            s=self.probe.messages['lowstate']
            mode=(int(s.mode_machine),int(s.mode_pr))
        if mode!=self.baseline:
            self.release_required=True
            raise Halt('lowstate mode fields changed; stop control')
        self.last_ok=time.monotonic()

    def check(self):
        if self.failure:raise Halt(self.failure)
        if time.monotonic()-self.last_ok>1.5:raise Halt('mode watchdog stale')

    def start(self):
        def loop():
            while not self.stop_event.is_set():
                try:self._query()
                except Exception as exc:
                    self.failure=str(exc)
                    if self.on_fault:self.on_fault()
                    return
                self.stop_event.wait(.2)
        self.thread=threading.Thread(target=loop,name='r1-debug-mode-guard',daemon=True)
        self.thread.start()

    def close(self):
        self.stop_event.set()
        if self.thread:self.thread.join(timeout=1)
