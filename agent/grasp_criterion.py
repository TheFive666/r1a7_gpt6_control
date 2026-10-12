"""Shared operator-defined wrist alignment criterion, not a motion planner."""
WRIST_GRASP_CRITERION = '''For the current yellow cake, the operator's wrist-image
closure criterion is: (1) the cake's intended grasp area lies between the two
black right Dex1 fingers; (2) both finger tips are approximately at the same image
height as that grasp area. Judge this from the fresh wrist_right RGB, not a
metric depth estimate. Equal left/right gaps, perfect centering, and visible air
on both sides are NOT required. Intended finger-to-cake contact is allowed.
Report cake_between_fingers and tips_at_grasp_height explicitly and honestly.
This criterion permits only stationary closure, not arbitrary arm translation.
Visible human intrusion, finger/table collision, impossible aperture, target
loss, stale images, or inability to judge these two features still blocks closure.
If the criterion is satisfied while waiting for wrist handover, hold stationary
for the next fresh observation rather than inventing another alignment motion.
For hold/pregrasp, assess clearance of the stationary pose, not a hypothetical
forward sweep. For move, the full proposed short motion must still be clear.
'''
