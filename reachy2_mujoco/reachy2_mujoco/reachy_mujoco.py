import os
import numpy as np
import cv2
import threading
import time

import logging

import mujoco
import mujoco.viewer

from .parts.joints import Tripod
from .parts import Arm, Cameras, Head, MobileBase
from .parts.cameras import CameraView
from reachy2_symbolic_ik.control_ik import ControlIK

os.environ['MUJOCO_GL'] = 'egl'
logger = logging.getLogger(__name__)
logging.basicConfig()
logger.setLevel(logging.INFO)
# with freejoint
# qpos = [x, y, z, qw, qx, qy, qz]
# qvel = [vx, vy, vz, ωx, ωy, ωz]


class ReachyMujoco:
    def __init__(self,env):


        self._urdf_path = "/".join(os.path.realpath(__file__).split("/")[:-2]) + "/description/modified_urdf/reachy2.urdf"


        if env =='':
            self._scene_path = "/".join(os.path.realpath(__file__).split("/")[:-2]) + "/description/mjcf/table_scene.xml"
        else:
            self._scene_path=env
            logger.warning(f"Loading environnement: {self._scene_path}")


        self._model = mujoco.MjModel.from_xml_path(self._scene_path)
        self._model.opt.timestep = 0.001
        self._data = mujoco.MjData(self._model)

        self._control_ik = ControlIK(urdf_path=self._urdf_path, current_joints=[[0] * 7, [0] * 7])

        mujoco.mj_step(self._model, self._data)

        self._state_size = mujoco.mj_stateSize(self._model,mujoco.mjtState.mjSTATE_INTEGRATION) #mjSTATE_INTERGRATION=everything

        self._init_state = np.empty(self._state_size, np.float64)
        mujoco.mj_getState(self._model, self._data, self._init_state, mujoco.mjtState.mjSTATE_INTEGRATION)


        self.l_arm = Arm(self._model, self._data, self._control_ik, prefix="l_")
        self.r_arm = Arm(self._model, self._data, self._control_ik, prefix="r_")
        self.head = Head(self._model, self._data)
        self.mobile_base = MobileBase(self._model, self._data)
        self.tripod=Tripod(self._model,self._data)
        self.turn_off()
        self.thread = threading.Thread(target=self._run)
        self.thread.start()

        self.cameras = None  # must be initialized after the viewer

    def reset_simulation(self, keyframe=''):
        """
        Reset the simulation
        """
        mujoco.mj_setState(self._model, self._data, self._init_state, mujoco.mjtState.mjSTATE_INTEGRATION)

        if keyframe != '':
            init_q = np.array(self._model.keyframe(keyframe).qpos)
            init_ctrl = self._model.keyframe(
                keyframe
            ).ctrl  # ctrl of all the actual joints (no floating base and no backlash)

            mujoco.mj_setState(self._model, self._data, init_q, mujoco.mjtState.mjSTATE_QPOS)
            mujoco.mj_setState(self._model, self._data, init_ctrl, mujoco.mjtState.mjSTATE_CTRL)

        self.l_arm._reset()
        self.r_arm._reset()
        self.head._reset()
        self.tripod._reset()
        self.mobile_base._reset()
        logger.info("Resetting simulation")

    def turn_on(self):
        return self.l_arm.turn_on() and self.r_arm.turn_on() and self.head.turn_on()

    def turn_off(self):
        return self.l_arm.turn_off() and self.r_arm.turn_off() and self.head.turn_off()


    def turn_off_smoothly(self):
        """
        Does nothing, is here just to match the real Reachy API.
        """
        pass

    def _update(self):
        self.mobile_base._update()
        if self.cameras is None:
            self.cameras = Cameras(self._model, self._data, 640, 480)
        else:
            self.cameras._update()
            # left = self.cameras.teleop.get_frame(view=CameraView.LEFT)
            # right = self.cameras.teleop.get_frame(view=CameraView.DEPTH)
            # cv2.imshow("left", left)
            # cv2.imshow("right", right)
            # cv2.waitKey(1)

    def _run(self):
        with mujoco.viewer.launch_passive(self._model, self._data, show_left_ui=False, show_right_ui=False) as viewer:
            i = 0
            while True:
                mujoco.mj_step(self._model, self._data, 20)  # 4 seems good
                self._update()
                viewer.sync()
                # time.sleep(1 / 500)
                i += 1

    def send_goal_positions(self, check_positions=False):
        self.l_arm._update()
        self.r_arm._update()
        self.tripod._update()
        self.head._update()
