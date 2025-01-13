import numpy as np
import time
from reachy2_mujoco import ReachySDK, CameraView

reachy = ReachySDK("localhost")


# test = np.eye(4)
# test[1, 3] = 0.3
# test[2, 3] = -0.5
# print(test)

elbow_90 = np.array(
    [[-0.015, -0.001, -1.0, 0.384], [-0.086, 0.996, 0.001, 0.224], [0.996, 0.086, -0.015, -0.273], [0.0, 0.0, 0.0, 1.0]]
)

reachy.l_arm.shoulder.roll.goal_position = 40
reachy.send_goal_positions()
time.sleep(1)
reachy.l_arm.shoulder.pitch.goal_position = -40
reachy.send_goal_positions()
while True:
    pos = reachy.l_arm.get_present_positions()
    print(np.around(np.array(pos)))
# exit()

# print(reachy.l_arm._control_ik)
reachy.l_arm.goto(elbow_90)
reachy.send_goal_positions()


# while True:
#     time.sleep(1 / 60)
