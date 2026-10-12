"""Offline regressions: joint publication must not imply Cartesian completion."""
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from agent.astra_policy import Halt
from agent.astra_robot import RightOnlyController


class EefCompletionTests(unittest.TestCase):
    def test_delta_is_sent_directly_to_teleop_placo(self):
        c = self.controller(1.)
        c.check_workspace = Mock()
        c.placo_solver = Mock()
        solved = np.array([0., 0., .02, 0., 0., 0., 0.])
        c.placo_solver.solve.return_value = solved
        c._validated_solution = Mock(return_value=solved)
        result = RightOnlyController.solve_step(c, SimpleNamespace(
            translation=(0., 0., .02), rotation=(0., 0., 0.)))
        c.placo_solver.solve.assert_called_once()
        args = c.placo_solver.solve.call_args.args
        np.testing.assert_array_equal(args[0], np.zeros(7))
        np.testing.assert_allclose(args[1]['right'][0], [0, 0, .02])
        np.testing.assert_allclose(args[1]['right'][1], [0, 0, 0, 1])
        self.assertIs(result, solved)

    def test_placo_failure_does_not_switch_solver(self):
        c = self.controller(1.)
        c.check_workspace = Mock()
        c.placo_solver = Mock()
        c.placo_solver.solve.side_effect = RuntimeError('QP failed')
        with self.assertRaisesRegex(Halt, 'Placo IK failed'):
            RightOnlyController.solve_step(c, SimpleNamespace(
                translation=(0., 0., .02), rotation=(0., 0., 0.)))

    def test_refinement_keeps_one_target_and_does_not_publish(self):
        c = self.controller(1.)
        c.check_workspace = Mock()
        first = np.array([0., 0., .01, 0., 0., 0., 0.])
        final = np.array([0., 0., .02, 0., 0., 0., 0.])
        c.placo_solver = Mock()
        c.placo_solver.solve.side_effect = [first, final]
        c._validated_solution = Mock(side_effect=[Halt('IK residual too large'), final])
        c._output_command = Mock()
        result = RightOnlyController.solve_step(c, SimpleNamespace(
            translation=(0., 0., .02), rotation=(0., 0., 0.)))
        self.assertIs(result, final)
        for call in c.placo_solver.solve.call_args_list:
            np.testing.assert_array_equal(call.args[1]['right'][0], [0, 0, .02])
        np.testing.assert_array_equal(c.q, np.zeros(7))
        c._output_command.assert_not_called()

    def controller(self, response):
        c = SimpleNamespace(q=np.zeros(7), right_arm_q_indices=list(range(7)),
                            position_checks=False, motion_mode=True, right_ratio=1.)
        c.pose = lambda q=None: ((c.q if q is None else q)[:3].copy(), np.array([0., 0., 0., 1.]))
        c.measured_pose = lambda: c.pose(np.zeros(7))
        c.refresh = lambda: None
        real = SimpleNamespace(lock=threading.Lock(), command_q=np.zeros(14))
        c.arm_ctrl = SimpleNamespace(real=real,
            get_current_dual_arm_q=lambda: np.r_[np.zeros(7), c.q * response])
        c._output_command = lambda: real.command_q.__setitem__(slice(7, 14), c.q)
        return c

    def execute(self, c):
        clock = [0.]
        def sleep(dt):
            clock[0] += max(dt, .001)
        target = np.array([0., 0., .02, 0., 0., 0., 0.])
        with patch('teleop.astra_robot.time.monotonic', side_effect=lambda: clock[0]), \
             patch('teleop.astra_robot.time.sleep', side_effect=sleep):
            RightOnlyController.execute_one(c, target, lambda: None, timeout=.8)

    def test_published_target_without_eef_motion_fails(self):
        c = self.controller(0.)
        with self.assertRaisesRegex(Halt, 'failed to settle'):
            self.execute(c)
        self.assertNotEqual(c.last_execution.get('outcome'), 'reached')

    def test_half_distance_is_not_reached(self):
        c = self.controller(.5)
        with self.assertRaisesRegex(Halt, 'failed to settle'):
            self.execute(c)

    def test_measured_target_succeeds(self):
        c = self.controller(1.)
        self.execute(c)
        self.assertEqual(c.last_execution['outcome'], 'reached')
        np.testing.assert_allclose(c.last_execution['measured_delta_m'], [0, 0, .02])

    def test_ik_residual_is_independent_of_hardware_load(self):
        c = self.controller(1.)
        c.model = object()
        c.check_workspace = lambda p: None
        with patch('teleop.astra_robot.clamp_configuration', side_effect=lambda model, q: q):
            with self.assertRaisesRegex(Halt, 'IK residual'):
                RightOnlyController._validated_solution(c, np.zeros(7), np.zeros(7),
                    np.array([0., 0., .02]), [0, 0, 0, 1], .0015, .01)


if __name__ == '__main__':
    unittest.main()
