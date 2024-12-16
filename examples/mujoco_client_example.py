import rpyc
import numpy as np
import time
# from reachy2_mujoco.server import ReachyMujoco  # Not necessary, for  autocompletion
# from reachy2_mujoco import ReachyMujoco
from reachy2_mujoco import ReachySDK

# conn = rpyc.connect("localhost", port=18861)
# reachy: ReachyMujoco = conn.root.reachy

reachy = ReachySDK("localhost")

reachy.mobile_base.goto(-1.0, 1.0, -np.pi / 2)

while True:
    print(reachy.mobile_base.position)
    # pos = 0.5 * np.sin(2 * np.pi * 0.5 * time.time())
    # reachy.r_arm.shoulder_pitch.goal_position = pos
    # reachy.send_goal_positions()
    # print(reachy.r_arm.shoulder_pitch.present_position)
    # # reachy.head.neck.yaw.set_goal_position(pos)
    time.sleep(1 / 60)
