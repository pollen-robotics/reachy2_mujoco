import rpyc
import numpy as np
import time
from reachy_mujoco import ReachyMujoco

conn = rpyc.connect("localhost", port=18861)
reachy: ReachyMujoco = conn.root.reachy


while True:
    pos = 0.5 * np.sin(2*np.pi*0.5*time.time())
    reachy.r_arm.shoulder_pitch.goal_position = pos
    reachy.send_goal_positions()
    print(reachy.r_arm.shoulder_pitch.present_position)
    # reachy.head.neck.yaw.set_goal_position(pos)
    time.sleep(1/60)