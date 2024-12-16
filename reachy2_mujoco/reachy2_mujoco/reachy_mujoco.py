import mujoco
import pyquaternion

# import cv2
import threading
import os
import mujoco.viewer
import time
import numpy as np

# import os
# os.environ['MUJOCO_GL'] = 'egl'

# with freejoint
# qpos = [x, y, z, qw, qx, qy, qz]
# qvel = [vx, vy, vz, ωx, ωy, ωz]


def get_actuator_name(model, index: int) -> str:
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)


def get_actuator_index(model, name: str) -> int:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)


class Joint:
    def __init__(self, model, data, name):
        self._model = model
        self._data = data
        self._name = name
        self._index = get_actuator_index(self._model, self._name)

        self.goal_position = 0  # expects degrees ?

    @property
    def present_position(self):
        return self._data.qpos[self._index]

    def _update(self):
        self._data.ctrl[self._index] = self.goal_position


class Gripper(Joint):
    def __init__(self, model, data, prefix="l_"):
        super().__init__(model, data, name=f"{prefix}gripper")

    def set_opening(self, percentage):
        print("Not implemented")
        pass


class Arm:
    def __init__(self, model, data, prefix="l_"):
        self._model = model
        self._data = data

        self.shoulder_pitch = Joint(self._model, self._data, f"{prefix}shoulder_pitch")
        self.shoulder_roll = Joint(self._model, self._data, f"{prefix}shoulder_roll")
        self.elbow_yaw = Joint(self._model, self._data, f"{prefix}elbow_yaw")
        self.elbow_pitch = Joint(self._model, self._data, f"{prefix}elbow_pitch")
        self.wrist_roll = Joint(self._model, self._data, f"{prefix}wrist_roll")
        self.wrist_pitch = Joint(self._model, self._data, f"{prefix}wrist_pitch")
        self.wrist_yaw = Joint(self._model, self._data, f"{prefix}wrist_yaw")
        self.gripper = Gripper(self._model, self._data, prefix=prefix)

    def _update(self):
        self.shoulder_pitch._update()
        self.shoulder_roll._update()
        self.elbow_yaw._update()
        self.elbow_pitch._update()
        self.wrist_roll._update()
        self.wrist_pitch._update()
        self.wrist_yaw._update()
        self.gripper._update()

    # TODO implement
    def goto(self):
        print("Not implemented")
        pass


class Neck:
    def __init__(self, model, data):
        self._model = model
        self._data = data

        self.roll = Joint(self._model, self._data, "neck_roll")
        self.pitch = Joint(self._model, self._data, "neck_pitch")
        self.yaw = Joint(self._model, self._data, "neck_yaw")

    def _update(self):
        self.roll._update()
        self.pitch._update()
        self.yaw._update()


class Head:
    def __init__(self, model, data):
        self._model = model
        self._data = data

        self.neck = Neck(self._model, self._data)

    def _update(self):
        self.neck._update()


class Camera:
    def __init__(self, model, data, cam_name, width, height):
        self._model = model
        self._data = data
        self._cam_name = cam_name
        self._width = width
        self._height = height
        self._camera_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_CAMERA, self._cam_name)
        self._offscreen = mujoco.MjrContext(self._model, mujoco.mjtFontScale.mjFONTSCALE_150)
        self._rgb_array = np.zeros((self._height, self._width, 3), dtype=np.uint8)

    def get_image(self):
        mujoco.mjv_updateScene(self._model, self._data, self._data.scn, self._data.cam)
        mujoco.mjr_render(
            mujoco.MjrRect(0, 0, self._width, self._height),
            self._data.scn,
            self._offscreen,
        )
        mujoco.mjr_readPixels(self._rgb_array, None, self._offscreen)
        return self._rgb_array


class MobileBase:
    # TODO implement
    # self.reachy.mobile_base.set_goal_speed(action[19], action[20], action[21])
    # self.reachy.mobile_base.send_speed_command()
    # self.reachy.mobile_base.last_cmd_vel
    # self.reachy.mobile_base.odometry
    def __init__(self, model, data):
        self._model = model
        self._data = data
        self._target_position = np.zeros(3)
        self._pos_offset = np.zeros(3)

        self.position = np.zeros(3)  # x, y, theta

    def _update_position(self):
        self.position[:2] = self._data.qpos[:2] - self._pos_offset[:2]

        quat = self._data.qpos[3:7]
        yaw = pyquaternion.Quaternion(quat).yaw_pitch_roll[0]

        self.position[2] = yaw - self._pos_offset[2]

    def _update(self):
        self._update_position()

        vector_2d = np.array(self._target_position[:2]) - self.position[:2]
        self._data.qvel[:2] = vector_2d

        # theta
        self._data.qvel[5] = self._target_position[2] - self.position[2]

    def reset_odometry(self):
        self._pos_offset = self.position

    # TODO implement better (with PID ?)
    def goto(self, x, y, theta):
        self._target_position = np.array([x, y, theta])

    # TODO match real behvior (command for only 200ms ? )
    def set_goal_speed(self, vx, vy, vtheta):
        self._data.qvel[:3] = [vx, vy, vtheta]


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

        # self.camera = Camera(self._model, self._data, "left_teleop_cam", 640, 480)

    def _list_actuators(self):
        for i in range(20):
            name = get_actuator_name(self._model, i)
            print(i, name)

    def _update(self):
        self.mobile_base._update()

        # im = self.camera.get_image()
        # cv2.imshow("image", im)
        # cv2.waitKey(1)
        pass

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
