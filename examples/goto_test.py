import numpy as np
import time

from reachy2_mujoco import ReachySDK, CameraView

reachy = ReachySDK("localhost")


# test = np.eye(4)
# test[1, 3] = 0.3
# test[2, 3] = -0.5
# print(test)

elbow_90 = np.array(
    [[-0.015, -0.001, -1.0, 0.384], [-0.086, 0.996, 0.001, 0.224], [0.996, 0.086, -0.03, -0.273], [0.0, 0.0, 0.0, 1.0]]
)

reachy.l_arm.shoulder.roll.goal_position = 40
reachy.r_arm.shoulder.roll.goal_position = -40
reachy.send_goal_positions()
time.sleep(1)

reachy.l_arm.goto(elbow_90)
# reachy.send_goal_positions()

time.sleep(2)

up = elbow_90.copy()
up[:3, 3] += np.array([0.2, 0, 0])

while True:
    reachy.l_arm.goto(up)
    time.sleep(2)
    reachy.l_arm.goto(elbow_90)
    time.sleep(2)



# while True:
#     time.sleep(1 / 60)
