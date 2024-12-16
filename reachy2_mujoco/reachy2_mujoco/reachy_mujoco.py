import os
import threading
import time

import mujoco
import mujoco.viewer

from reachy2_mujoco.parts import Arm, Camera, Head, MobileBase
from reachy2_mujoco.utils import get_actuator_name

# import os
# os.environ['MUJOCO_GL'] = 'egl'

# with freejoint
# qpos = [x, y, z, qw, qx, qy, qz]
# qvel = [vx, vy, vz, ωx, ωy, ωz]


class ReachyMujoco:
    def __init__(self):
        scene_path = "/".join(os.path.realpath(__file__).split("/")[:-2]) + "/description/mjcf/scene.xml"

        self._model = mujoco.MjModel.from_xml_path(scene_path)
        self._model.opt.timestep = 0.001
        self._data = mujoco.MjData(self._model)
        mujoco.mj_step(self._model, self._data)

        self.l_arm = Arm(self._model, self._data, prefix="l_")
        self.r_arm = Arm(self._model, self._data, prefix="r_")
        self.head = Head(self._model, self._data)
        self.mobile_base = MobileBase(self._model, self._data)

        self.thread = threading.Thread(target=self._run)
        self.thread.start()

        # self._list_actuators()
        # exit()

        self.camera = None  # must be initialized after the viewer

    def turn_on(self):
        pass

    def turn_off(self):
        pass

    def turn_off_smoothly(self):
        pass

    def _list_actuators(self):
        for i in range(20):
            name = get_actuator_name(self._model, i)
            print(i, name)

    def _update(self):
        self.mobile_base._update()
        if self.camera is None:
            self.camera = Camera(self._model, self._data, "left_teleop_cam", 320, 240)
        else:
            self.camera._update()
        # else:
        #     im = self.camera.get_image()
        #     cv2.imshow("image", im)
        #     cv2.waitKey(1)

    def _run(self):
        with mujoco.viewer.launch_passive(self._model, self._data, show_left_ui=False, show_right_ui=False) as viewer:
            i = 0
            while True:
                mujoco.mj_step(self._model, self._data, 7)  # 4 seems good
                self._update()
                viewer.sync()
                time.sleep(1 / 60)
                i += 1

    def send_goal_positions(self, check_positions=False):
        self.l_arm._update()
        self.r_arm._update()
        self.head._update()
