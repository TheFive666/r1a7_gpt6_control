"""Detect cached camera slots inside otherwise fresh four-camera packets."""
import time
import numpy as np
from agent.astra_policy import Halt

class VisualFreshness:
    def __init__(self, timeout=2.0):
        self.timeout=timeout
        self.last_id=None
        self.previous={}
        self.changed_at={}

    def update(self, frames, frame_id, now=None):
        now=time.monotonic() if now is None else now
        if frame_id!=self.last_id:
            for name,index in (('head_right',1),('wrist_right',3)):
                frame=frames[index]
                if name not in self.previous or not np.array_equal(frame,self.previous[name]):
                    self.changed_at[name]=now
                    self.previous[name]=frame.copy()
            self.last_id=frame_id
        return {name:now-stamp for name,stamp in self.changed_at.items()}

    def require_live(self, frames, frame_id, now=None):
        ages=self.update(frames,frame_id,now)
        frozen=[name for name,age in ages.items() if age>self.timeout]
        if frozen:raise Halt('unchanged camera pixels / possible cached image: '+', '.join(frozen))
        return ages
