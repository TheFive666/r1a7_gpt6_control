import unittest
from agent.astra_grasp_stage import validate_evidence
from agent.astra_policy import Halt

class EvidenceTests(unittest.TestCase):
    def evidence(self):
        return dict(cake_between_fingers=True,tips_at_grasp_height=True,safe_to_close=True,safe_to_lift=True,object_follows_hand=True,
                    object_clear_of_table=True,uncertain=False,confidence=.9,reason='visible capture')
    def test_close_requires_geometry_not_just_target_visibility(self):
        raw=self.evidence();raw['safe_to_close']=False
        with self.assertRaises(Halt):validate_evidence(raw,['safe_to_close'])
    def test_follow_and_table_separation_are_distinct(self):
        raw=self.evidence();raw['object_clear_of_table']=False
        validate_evidence(raw,['object_follows_hand'])
        with self.assertRaises(Halt):validate_evidence(raw,['object_follows_hand','object_clear_of_table'])
    def test_uncertainty_and_nonfinite_confidence_block(self):
        for key,value in [('uncertain',True),('confidence',float('nan')),('confidence',.79),('object_follows_hand',1)]:
            raw=self.evidence();raw[key]=value
            with self.assertRaises(Halt):validate_evidence(raw,['object_follows_hand'])

    def test_operator_alignment_both_features_required(self):
        for key in ('cake_between_fingers','tips_at_grasp_height'):
            raw=self.evidence();raw[key]=False
            with self.assertRaises(Halt):
                validate_evidence(raw,['safe_to_close','cake_between_fingers','tips_at_grasp_height'])

    def test_trial_lift_does_not_require_future_follow_evidence(self):
        raw=self.evidence();raw['object_follows_hand']=False;raw['object_clear_of_table']=False
        validate_evidence(raw,['safe_to_lift'])
        with self.assertRaises(Halt):validate_evidence(raw,['object_follows_hand'])
