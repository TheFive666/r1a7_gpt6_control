"""Interpolate one validated IK target; never extrapolate policy actions."""
import numpy as np

CONTROL_HZ = 30.0
CONTROL_DT = 1.0 / CONTROL_HZ


def joint_step_samples(start, goal, right_indices):
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    if start.shape != goal.shape or not np.all(np.isfinite([start, goal])):
        raise ValueError('invalid interpolation endpoints')
    fixed = np.ones(start.size, dtype=bool)
    fixed[right_indices] = False
    if not np.array_equal(start[fixed], goal[fixed]):
        raise ValueError('interpolation cannot move inactive joints')
    # At least 0.4 s per visual action, and <=0.004 rad per 30 Hz tick.
    count = max(12, int(np.ceil(np.max(np.abs(goal - start)) / .004)))
    samples = []
    for i in range(1, count + 1):
        q = start.copy()
        q[right_indices] = start[right_indices] + (goal[right_indices] - start[right_indices]) * (i / count)
        samples.append(q)
    samples[-1] = goal.copy()
    return samples
