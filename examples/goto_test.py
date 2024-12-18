import numpy as np
import time
from reachy2_mujoco import ReachySDK, CameraView

reachy = ReachySDK("localhost")


test = np.eye(4)
test[1, 3] = 0.3
test[2, 3] = -0.5
print(test)


# print(reachy.l_arm._control_ik)
reachy.l_arm.goto(list(test))
reachy.send_goal_positions()
