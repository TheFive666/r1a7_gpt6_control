"""Bounded partial-response classification for millimetre visual corrections."""
import numpy as np


def partial_response(expected, measured, rotation_error, joint_error, stable_span):
    expected, measured = np.asarray(expected), np.asarray(measured)
    distance = float(np.linalg.norm(expected))
    if not 0.001 <= distance <= 0.00300001:
        return False
    progress = float(np.dot(measured, expected) / distance**2)
    lateral = float(np.linalg.norm(measured - progress * expected))
    return bool(
        np.all(np.isfinite(measured)) and 0.25 <= progress <= 1.0
        and lateral <= 0.001 and np.linalg.norm(measured - expected) <= 0.002
        and rotation_error <= 0.005 and joint_error <= 0.02
        and stable_span <= 0.0005)
