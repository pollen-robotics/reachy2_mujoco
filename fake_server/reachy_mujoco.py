import mujoco
import cv2
import threading
import mujoco.viewer
import time
import numpy as np


# TODO mobile base


def get_actuator_name(model, index: int) -> str:
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)


def get_actuator_index(model, name: str) -> int:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)


class Joint:
    def __init__(self, model, data, name):
        self.model = model
        self.data = data
        self.name = name
        self.index = get_actuator_index(self.model, self.name)

    @property
    def exposed_present_position(self):
        return self.data.qpos[self.index]

    @property
    def exposed_goal_position(self):
        return self.data.ctrl[self.index]

    def exposed_set_goal_position(self, position):
        self.data.ctrl[self.index] = position


class Gripper(Joint):
    def __init__(self, model, data, prefix="l_"):
        super().__init__(model, data, name=f"{prefix}gripper")

    def exposed_set_opening(self, percentage):
        print("Not implemented")
        pass


class Arm:
    def __init__(self, model, data, prefix="l_"):
        self.model = model
        self.data = data
        self.exposed_shoulder_pitch = Joint(
            self.model, self.data, f"{prefix}shoulder_pitch"
        )
        self.exposed_shoulder_roll = Joint(
            self.model, self.data, f"{prefix}shoulder_roll"
        )
        self.exposed_elbow_yaw = Joint(self.model, self.data, f"{prefix}elbow_yaw")
        self.exposed_elbow_pitch = Joint(self.model, self.data, f"{prefix}elbow_pitch")
        self.exposed_wrist_roll = Joint(self.model, self.data, f"{prefix}wrist_roll")
        self.exposed_wrist_pitch = Joint(self.model, self.data, f"{prefix}wrist_pitch")
        self.exposed_wrist_yaw = Joint(self.model, self.data, f"{prefix}wrist_yaw")
        self.exposed_gripper = Gripper(self.model, self.data, prefix=prefix)


class Neck:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.exposed_roll = Joint(self.model, self.data, "neck_roll")
        self.exposed_pitch = Joint(self.model, self.data, "neck_pitch")
        self.exposed_yaw = Joint(self.model, self.data, "neck_yaw")


class Head:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.exposed_neck = Neck(self.model, self.data)


class Camera:
    def __init__(self, model, data, cam_name, width, height):
        self.model = model
        self.data = data
        self.cam_name = cam_name
        self.width = width
        self.height = height
        self.camera_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.cam_name
        )
        self.offscreen = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)
        self.rgb_array = np.zeros((self.height, self.width, 3), dtype=np.uint8)

    def get_image(self):
        mujoco.mjv_updateScene(self.model, self.data, self.data.scn, self.data.cam)
        mujoco.mjr_render(
            mujoco.MjrRect(0, 0, self.width, self.height), self.data.scn, self.offscreen
        )
        mujoco.mjr_readPixels(self.rgb_array, None, self.offscreen)
        return self.rgb_array


class ReachyMujoco:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path("mjcf/scene.xml")
        self.model.opt.timestep = 0.001
        self.data = mujoco.MjData(self.model)
        mujoco.mj_step(self.model, self.data)

        self.exposed_l_arm = Arm(self.model, self.data, prefix="l_")
        self.exposed_r_arm = Arm(self.model, self.data, prefix="r_")
        self.exposed_head = Head(self.model, self.data)

        # for i in range(20):
        #     name = get_actuator_name(self.model, i)
        #     print(i, name)

        # self.camera = Camera(self.model, self.data, "left_teleop_cam", 640, 480)

        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def update(self):
        # im = self.camera.get_image()
        # cv2.imshow("image", im)
        # cv2.waitKey(1)
        pass

    def run(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            i = 0
            while True:
                mujoco.mj_step(self.model, self.data, 7)  # 4 seems good
                self.update()
                viewer.sync()
                time.sleep(1 / 60)
                i += 1

    def get_actuator_index(self, name: str) -> int:
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
