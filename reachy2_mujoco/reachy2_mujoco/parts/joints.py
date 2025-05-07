from ..utils import get_actuator_index, get_joint_index
import numpy as np
import mujoco

class Joint:
    def __init__(self, model, data, name):
        self._model = model
        self._data = data
        self._name = name
        self._ctrl_index = get_actuator_index(self._model, self._name)
        self._jnt_index= get_joint_index(self._model, self._name)
        # self._qpos_index = self._model.actuator(self._name).trnid[0]

        joint_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, self._name)
        self._qpos_index = self._model.jnt_qposadr[joint_id]

        print(f"Joint {self._name} has ctrl index {self._ctrl_index} and qpos index {self._qpos_index}")

        self.goal_position = 0  # expects degrees

    @property
    def present_position(self):
        return np.rad2deg(self._data.qpos[self._qpos_index])

    def _update(self):
        self._data.ctrl[self._ctrl_index] = self.goal_position * np.pi / 180

        # print(f"{self._name} present_position : {self.present_position}")


class Shoulder:
    def __init__(self, model, data, prefix="l_"):
        self._model = model
        self._data = data
        self.pitch = Joint(self._model, self._data, f"{prefix}shoulder_pitch")
        self.roll = Joint(self._model, self._data, f"{prefix}shoulder_roll")

    def _update(self):
        self.pitch._update()
        self.roll._update()


class Elbow:
    def __init__(self, model, data, prefix="l_"):
        self._model = model
        self._data = data
        self.yaw = Joint(self._model, self._data, f"{prefix}elbow_yaw")
        self.pitch = Joint(self._model, self._data, f"{prefix}elbow_pitch")

    def _update(self):
        self.yaw._update()
        self.pitch._update()


class Wrist:
    def __init__(self, model, data, prefix="l_"):
        self._model = model
        self._data = data
        self.roll = Joint(self._model, self._data, f"{prefix}wrist_roll")
        self.pitch = Joint(self._model, self._data, f"{prefix}wrist_pitch")
        self.yaw = Joint(self._model, self._data, f"{prefix}wrist_yaw")

    def _update(self):
        self.roll._update()
        self.pitch._update()
        self.yaw._update()


class Gripper(Joint):
    def __init__(self, model, data, prefix="l_"):
        super().__init__(model, data, name=f"{prefix}hand_finger")
        self._limits = self._model.jnt_range[self._jnt_index]
        self.open()
        # self._limits = [-100, 100] # TODO fix

    def set_opening(self, percentage):
        self.goal_position = np.rad2deg(np.interp(percentage, [0, 100], self._limits))

    def open(self):
        self.set_opening(100)
        self._update()

    def close(self):
        self.set_opening(0)
        self._update()


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
