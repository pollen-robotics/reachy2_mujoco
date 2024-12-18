import cv2
import numpy as np
import time
from reachy2_mujoco import ReachySDK, CameraView

reachy = ReachySDK("localhost")

# reachy.mobile_base.goto(-1.0, 1.0, -np.pi / 2)

reachy.l_arm.shoulder.roll.goal_position = 40
# reachy.l_arm.elbow.pitch.goal_position = -90

reachy.r_arm.shoulder.roll.goal_position = -40
reachy.r_arm.elbow.pitch.goal_position = -90
while True:
    pos = 0.5 * np.sin(2 * np.pi * 0.2 * time.time())

    reachy.r_arm.shoulder.pitch.goal_position = np.rad2deg(pos)
    # reachy.l_arm.shoulder.pitch.goal_position = np.rad2deg(pos)

    gripper_opening = 100 if pos > 0 else 0
    reachy.r_arm.gripper.set_opening(gripper_opening)

    reachy.head.neck.roll.goal_position = np.rad2deg(pos)
    reachy.head.neck.pitch.goal_position = np.rad2deg(pos)
    reachy.head.neck.yaw.goal_position = np.rad2deg(pos)

    mobile_base_pos = [-1, -1, -np.pi / 2] if pos > 0 else [0, 0, 0]
    reachy.mobile_base.goto(*mobile_base_pos)

    reachy.send_goal_positions()
    im = np.array(reachy.cameras.teleop.get_frame(view=CameraView.LEFT))
    cv2.imshow("image", im)
    cv2.waitKey(1)

    time.sleep(1 / 60)
