import rpyc
import numpy as np
import time

conn = rpyc.connect("localhost", port=18861)
reachy = conn.root.reachy


while True:
    pos = 0.5 * np.sin(2*np.pi*0.5*time.time())
    reachy.r_arm.shoulder_pitch.set_goal_position(pos)
    # reachy.head.neck.yaw.set_goal_position(pos)

    time.sleep(1/60)