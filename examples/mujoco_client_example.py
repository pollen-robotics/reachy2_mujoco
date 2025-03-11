import cv2
import numpy as np
import time
from reachy2_mujoco import ReachySDK, CameraView

reachy = ReachySDK("localhost")

reachy.l_arm.shoulder.roll.goal_position = 90

reachy.r_arm.shoulder.roll.goal_position = -40
reachy.r_arm.elbow.pitch.goal_position = -90
while True:
    pos = 0.5 * np.sin(2 * np.pi * 0.05 * time.time())

    reachy.r_arm.shoulder.pitch.goal_position = np.rad2deg(pos)

    reachy.r_arm.gripper.set_opening(100 if pos > 0 else 0)

    # reachy.head.neck.roll.goal_position = np.rad2deg(pos)
    reachy.head.neck.pitch.goal_position = 30
    # reachy.head.neck.yaw.goal_position = np.rad2deg(pos)

    mobile_base_pos = [-1.0, -1.0, -np.pi / 2] if pos > 0 else [0, 0, 0]
    reachy.mobile_base.goto(*mobile_base_pos)

    reachy.send_goal_positions()
    left = np.array(reachy.cameras.teleop.get_frame(view=CameraView.LEFT))
    depth = np.array(reachy.cameras.teleop.get_frame(view=CameraView.DEPTH))
    cv2.imshow("depth", depth)
    cv2.imshow("left", left)
    cv2.waitKey(1)
    print(np.around(np.array(reachy.r_arm.get_present_positions())))

    time.sleep(1 / 60)
