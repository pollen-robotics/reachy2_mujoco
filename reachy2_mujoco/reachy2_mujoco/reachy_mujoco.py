import os
import cv2
import threading
import time

import mujoco
import mujoco.viewer

from reachy2_mujoco.parts import Arm, Cameras, Head, MobileBase
from reachy2_mujoco.parts.cameras import CameraView
from reachy2_symbolic_ik.control_ik import ControlIK

# import os
# os.environ['MUJOCO_GL'] = 'egl'

# with freejoint
# qpos = [x, y, z, qw, qx, qy, qz]
# qvel = [vx, vy, vz, ωx, ωy, ωz]


class ReachyMujoco:
    def __init__(self):
        self._scene_path = "/".join(os.path.realpath(__file__).split("/")[:-2]) + "/description/mjcf/table_scene.xml"
        self._urdf_path = "/".join(os.path.realpath(__file__).split("/")[:-2]) + "/description/modified_urdf/reachy2.urdf"

        self._model = mujoco.MjModel.from_xml_path(self._scene_path)
        self._model.opt.timestep = 0.001
        self._data = mujoco.MjData(self._model)

        self._control_ik = ControlIK(urdf_path=self._urdf_path, current_joints=[[0] * 7, [0] * 7])

        mujoco.mj_step(self._model, self._data)

        self.l_arm = Arm(self._model, self._data, self._control_ik, prefix="l_")
        self.r_arm = Arm(self._model, self._data, self._control_ik, prefix="r_")
        self.head = Head(self._model, self._data)
        self.mobile_base = MobileBase(self._model, self._data)

        self.thread = threading.Thread(target=self._run)
        self.thread.start()

        self.cameras = None  # must be initialized after the viewer

    def turn_on(self):
        """
        Does nothing, is here just to match the real Reachy API.
        """
        pass

    def turn_off(self):
        """
        Does nothing, is here just to match the real Reachy API.
        """
        pass

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
                time.sleep(1 / 500)
                i += 1

    def send_goal_positions(self, check_positions=False):
        self.l_arm._update()
        self.r_arm._update()
        self.head._update()
