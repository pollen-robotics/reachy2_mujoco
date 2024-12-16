import argparse
import json
import os
import time
from glob import glob

import mujoco
import mujoco_viewer
import numpy as np


model = mujoco.MjModel.from_xml_path("mjcf/scene.xml")
model.opt.timestep = 0.001
data = mujoco.MjData(model)
mujoco.mj_step(model, data)
viewer = mujoco_viewer.MujocoViewer(model, data)


def get_actuator_index(name: str) -> int:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)


while True:
    # data.qpos[:3] += [0.01, 0, 0]

    data.ctrl[get_actuator_index("neck_roll")] = np.sin(time.time() * 2)

    mujoco.mj_step(model, data, 7)  # 4 seems good
    viewer.render()
