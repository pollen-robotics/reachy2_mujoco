from reachy2_mujoco.utils import get_actuator_index
import numpy as np


class Joint:
    def __init__(self, model, data, name):
        self._model = model
        self._data = data
        self._name = name
        self._index = get_actuator_index(self._model, self._name)

        self.goal_position = 0  # expects degrees

    @property
    def present_position(self):
        return np.rad2deg(self._data.qpos[self._index])

    def _update(self):
        self._data.ctrl[self._index] = self.goal_position * np.pi / 180


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
        super().__init__(model, data, name=f"{prefix}gripper")

    def set_opening(self, percentage):
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
