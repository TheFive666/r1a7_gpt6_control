import unittest
import numpy as np
from agent.visual_freshness import VisualFreshness
from agent.astra_policy import Halt

class FreshnessTests(unittest.TestCase):
    def test_new_packet_does_not_make_cached_wrist_fresh(self):
        g=VisualFreshness();frames=[np.zeros((2,2,3),np.uint8) for _ in range(4)]
        g.require_live(frames,1,now=0)
        frames[0][:]=1;frames[1][:]=1
        with self.assertRaisesRegex(Halt,'wrist_right'):g.require_live(frames,2,now=3)
    def test_left_wrist_not_processed(self):
        g=VisualFreshness();frames=[np.zeros((2,2,3),np.uint8),np.zeros((2,2,3),np.uint8),None,np.zeros((2,2,3),np.uint8)]
        g.require_live(frames,1,now=0)
        for i in (1,3):frames[i][:]=1
        self.assertEqual(set(g.require_live(frames,2,now=3)),{'head_right','wrist_right'})
