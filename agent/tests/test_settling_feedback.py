import unittest
from agent.settling_feedback import partial_response


class PartialResponseTests(unittest.TestCase):
    def test_small_stable_forward_response_is_partial(self):
        self.assertTrue(partial_response([.002,-.002,0], [.00123,-.00052,-.00021], .002,.012,.0001))

    def test_no_progress_reverse_large_drift_or_tracking_error_is_not_partial(self):
        for measured, rotation, joint, span in [
            ([0,0,0],0,0,0), ([-.001,0,0],0,0,0),
            ([.001,.002,0],0,0,0), ([.001,0,0],.006,0,0),
            ([.001,0,0],0,.03,0), ([.001,0,0],0,0,.001)]:
            self.assertFalse(partial_response([.002,0,0],measured,rotation,joint,span))

    def test_transport_cannot_use_partial(self):
        self.assertFalse(partial_response([.02,0,0],[.015,0,0],0,0,0))
