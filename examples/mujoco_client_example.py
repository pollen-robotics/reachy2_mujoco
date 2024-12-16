import cv2
import numpy as np
import time
from reachy2_mujoco import ReachySDK, CameraView

reachy = ReachySDK("localhost")

reachy.mobile_base.goto(-1.0, 1.0, -np.pi / 2)

while True:
    print(reachy.mobile_base.position)

    pos = 0.5 * np.sin(2 * np.pi * 0.5 * time.time())
    reachy.r_arm.shoulder.pitch.goal_position = np.rad2deg(pos)
    reachy.send_goal_positions()
    im = np.array(reachy.cameras.teleop.get_frame(view=CameraView.DEPTH))
    cv2.imshow("image", im)
    cv2.waitKey(1)

    time.sleep(1 / 60)
