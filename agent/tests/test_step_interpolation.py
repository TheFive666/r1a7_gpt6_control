import unittest
import numpy as np

from agent.step_interpolation import CONTROL_DT, joint_step_samples


class InterpolationTests(unittest.TestCase):
    def test_right_only_bounded_exact_endpoint(self):
        start = np.arange(16, dtype=float) / 100
        goal = start.copy()
        right = list(range(7, 14))
        goal[right] += np.linspace(-.13, .21, 7)
        samples = joint_step_samples(start, goal, right)
        sequence = np.array([start] + samples)
        self.assertLessEqual(np.max(np.abs(np.diff(sequence, axis=0))), .004 + 1e-12)
        np.testing.assert_array_equal(samples[-1], goal)
        for q in samples:
            np.testing.assert_array_equal(q[:7], start[:7])
            np.testing.assert_array_equal(q[14:], start[14:])
        self.assertAlmostEqual(CONTROL_DT, 1 / 30)

    def test_rejects_inactive_change_and_nan(self):
        start = np.zeros(16)
        for index, value in [(0, .1), (14, .1), (7, float('nan'))]:
            goal = start.copy()
            goal[index] = value
            with self.assertRaises(ValueError):
                joint_step_samples(start, goal, list(range(7, 14)))

    def test_no_extrapolation(self):
        start = np.zeros(16)
        goal = start.copy()
        goal[7] = .001
        samples = joint_step_samples(start, goal, list(range(7, 14)))
        self.assertEqual(len(samples), 12)
        self.assertTrue(all(0 <= q[7] <= .001 for q in samples))


if __name__ == '__main__':
    unittest.main()
