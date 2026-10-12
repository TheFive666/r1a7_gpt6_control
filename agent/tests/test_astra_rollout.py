import copy
import unittest
from unittest.mock import Mock

import numpy as np

from agent.astra_policy import Halt, Phase, selected_views, validate_decision


def proposal(action='move'):
    return dict(action=action, confidence=0.99, target_visible=True, hand_visible=True,
                collision_free=True, uncertain=False, delta_base_m=[0.001, 0, 0],
                rotvec_base_rad=[0, 0, 0], reason='Visible clearance and alignment')


def stationary(action):
    p = proposal(action)
    p['delta_base_m'] = [0, 0, 0]
    return p


class PolicyTests(unittest.TestCase):
    def test_rejects_unsafe_or_unstructured_outputs(self):
        bad = []
        for k, value in [('confidence', .89), ('confidence', float('nan')), ('confidence', True),
                         ('uncertain', True), ('collision_free', False), ('target_visible', False),
                         ('hand_visible', False), ('target_visible', 'true'), ('action', 'close'),
                         ('delta_base_m', [float('nan'), 0, 0]), ('delta_base_m', [True, 0, 0]),
                         ('delta_base_m', [0.004, 0, 0]), ('rotvec_base_rad', [0, 0, .011])]:
            p = proposal(); p[k] = value; bad.append(p)
        p = proposal(); p['joints'] = [0]*7; bad.append(p)
        bad.append(stationary('move'))
        bad.append(proposal('pregrasp'))
        for p in bad:
            with self.subTest(p=p), self.assertRaises(Halt):
                validate_decision(p, 'wrist')

    def test_phase_requires_fresh_confirmations_from_caller(self):
        phase = Phase()
        with self.assertRaises(Halt):
            validate_decision(stationary('wrist_ready'), phase.stage)
        phase.accept(validate_decision(stationary('inspect_wrist'), phase.stage))
        for i in range(3):
            phase.accept(validate_decision(stationary('wrist_ready'), phase.stage))
            self.assertEqual(phase.stage, 'wrist' if i == 2 else 'wrist_check')
        self.assertFalse(phase.accept(validate_decision(stationary('pregrasp'), phase.stage)))
        phase.accept(validate_decision(proposal(), phase.stage))
        for i in range(3):
            self.assertEqual(phase.accept(validate_decision(stationary('pregrasp'), phase.stage)), i == 2)

    def test_left_wrist_never_selected(self):
        frames = (object(), object(), object(), object())
        for stage in ('head', 'wrist_check', 'wrist'):
            views = selected_views(frames, stage)
            self.assertFalse(any(f is frames[2] for _, f in views))
            self.assertEqual([n for n, _ in views], ['head_left', 'head_right'] if stage == 'head' else ['wrist_right'])

    def test_stereo_can_remain_primary_through_pregrasp(self):
        phase = Phase()
        for i in range(3):
            self.assertEqual(phase.accept(validate_decision(stationary('pregrasp'), phase.stage)), i == 2)
        phase = Phase()
        phase.accept(validate_decision(stationary('inspect_wrist'), 'head'))
        phase.accept(validate_decision(stationary('use_head'), 'wrist_check'))
        self.assertEqual(phase.stage, 'head')
        raw=stationary('use_head'); raw['target_visible']=False
        with self.assertRaises(Halt): validate_decision(raw,'wrist')


class RobotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from robot.config import R1A7Config, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, LEFT_ARM_INIT, RIGHT_ARM_INIT
        from agent.astra_robot import RightOnlyController
        joints = dict(zip(LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS, list(LEFT_ARM_INIT) + list(RIGHT_ARM_INIT)))
        joints.update(head_pitch_joint=0., head_yaw_joint=0., waist_yaw_joint=0.)
        cls.controller = RightOnlyController(R1A7Config(), dict(joint_commands=joints, left_closed_ratio=.4), [-.2,.8,-.85,.05,-.45,.7])

    def setUp(self):
        self.c = self.controller
        self.c.q = self.c.frozen.copy()
        self.c.hand_ctrl = None
        self.c.motion_mode = False
        self.c.arm_ctrl = Mock()

    def test_dls_without_elbow_returns_and_holds_other_joints(self):
        from teleop.ik_solver import solve_upper_arm_ik
        from scipy.spatial.transform import Rotation as R
        p, r = self.c.pose()
        q, diag = solve_upper_arm_ik(self.c.model, self.c.data, self.c.q,
                                    'right_dex1_base_link', p, R.from_quat(r).as_matrix(),
                                    None, None, self.c.right_arm_q_indices, self.c.q.copy(), 'pose')
        self.assertEqual(q.shape, self.c.q.shape)
        np.testing.assert_array_equal(q[self.c.fixed_indices], self.c.q[self.c.fixed_indices])
        self.assertIn('ee_pos_error_norm', diag)

    def test_actual_ik_small_step_is_right_only(self):
        decision = validate_decision(proposal(), 'wrist')
        q = self.c.solve_step(decision)
        np.testing.assert_array_equal(q[self.c.fixed_indices], self.c.frozen[self.c.fixed_indices])
        self.assertLessEqual(np.max(np.abs(q - self.c.q)), .04)
        self.c.execute_one(q, lambda: None)
        self.c.arm_ctrl.ctrl_dual_arm.assert_called_once()
        arms = self.c.arm_ctrl.ctrl_dual_arm.call_args.args[0]
        self.assertEqual(arms.shape, (14,))
        np.testing.assert_array_equal(arms[:7], self.c.frozen[self.c.left_arm_q_indices])

    def test_joint_limits_and_workspace_reject(self):
        with self.assertRaises(Halt):
            self.c.check_workspace(np.array([2, 0, 0]))
        q = self.c.q.copy(); q[self.c.right_arm_q_indices[0]] = 10
        p, r = self.c.pose()
        with self.assertRaisesRegex(Halt, 'limits'):
            self.c._validated_solution(q, self.c.q, p, r, .001, .01)

    def test_home_is_100_frame_right_only_blend(self):
        self.c.q[self.c.right_arm_q_indices[0]] += .1
        start = self.c.q.copy()
        self.c.reset()
        np.testing.assert_array_equal(self.c.q, start)
        for i in range(100):
            self.c.advance_home_blend()
            np.testing.assert_array_equal(self.c.q[self.c.fixed_indices], self.c.frozen[self.c.fixed_indices])
            expected = start[self.c.right_arm_q_indices] + ((i+1)/100) * (self.c.home_right - start[self.c.right_arm_q_indices])
            np.testing.assert_allclose(self.c.q[self.c.right_arm_q_indices], expected, atol=1e-12)
        self.assertFalse(self.c._reset_blending)
        self.assertEqual(self.c.arm_ctrl.ctrl_dual_arm.call_count, 100)

    def test_unchanged_left_gripper_and_open_right(self):
        self.c.hand_ctrl = Mock()
        self.c._output_command()
        self.c.hand_ctrl.set_gripper_ratios.assert_called_once_with(.4, 0.0)

    def test_feedback_does_not_erase_command_configuration(self):
        command = self.c.q.copy()
        measured = np.concatenate((command[self.c.left_arm_q_indices], command[self.c.right_arm_q_indices]))
        measured[10] += .01
        self.c.arm_ctrl.get_current_dual_arm_q.return_value = measured
        self.c.motion_mode = True
        try:
            self.c.refresh()
            np.testing.assert_array_equal(self.c.q, command)
            np.testing.assert_array_equal(self.c.measured_q[self.c.right_arm_q_indices], measured[7:])
            self.assertGreater(np.linalg.norm(self.c.measured_pose()[0] - self.c.pose()[0]), .0001)
        finally:
            self.c.motion_mode = False


class FrameTests(unittest.TestCase):
    def test_rejects_pre_action_and_duplicate_stereo(self):
        import time
        from agent.scripts.astra_grasp_rollout import fresh_frames
        sub = Mock()
        image = np.zeros((2,2,3), dtype=np.uint8)
        frames = (image, image + 1, image, image)
        sub.get_if_new.return_value = frames, 2, 10
        with self.assertRaises(Halt):
            fresh_frames(sub, 1, 20, timeout=.03)
        sub.get_if_new.return_value = (image, image, image, image), 3, time.time_ns()
        with self.assertRaisesRegex(Halt, 'duplicated'):
            fresh_frames(sub, 2, 0, timeout=.1)


if __name__ == '__main__':
    unittest.main()
