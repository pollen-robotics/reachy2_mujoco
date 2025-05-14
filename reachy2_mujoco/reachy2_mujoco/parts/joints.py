from ..utils import get_actuator_index, get_joint_index
import numpy as np
import mujoco
import copy

import logging
logger = logging.getLogger(__name__)
logging.basicConfig()
logger.setLevel(logging.INFO)

class Joint:
    def __init__(self, model, data, name):
        self._model = model
        self._data = data
        self._name = name
        self._ctrl_index = get_actuator_index(self._model, self._name)
        self._jnt_index= get_joint_index(self._model, self._name)
        # self._qpos_index = self._model.actuator(self._name).trnid[0]

        self._joint_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, self._name)
        self._qpos_index = self._model.jnt_qposadr[self._joint_id]
        self._frc_range=copy.copy(self._model.jnt_actfrcrange[self._joint_id])

        logger.debug(f"Joint {self._name} has ctrl index {self._ctrl_index} and qpos index {self._qpos_index}")

        self.goal_position = 0.0  # expects degrees
        self._on=False
        self.turn_off()

    @property
    def present_position(self):
        return np.rad2deg(self._data.qpos[self._qpos_index])

    def _update(self):

        self._data.ctrl[self._ctrl_index] = np.deg2rad(self.goal_position)

        # print(f"{self._name} present_position : {self.present_position}")
    def _reset(self):
        self.goal_position=self._data.ctrl[self._ctrl_index]

    def _turn_on(self):
        self._model.jnt_actfrcrange[self._joint_id]=self._frc_range

    def _turn_off(self):
        self._model.jnt_actfrcrange[self._joint_id]=[0.0,0.0]

    def turn_on(self):
        #TODO simulate speed_limit with damping?
        self._turn_on()
        self._on=True
        return True

    def turn_off(self):
        self._turn_off()
        self._on=False
        return True

    def is_on(self):
        return self._on

    def if_off(self):
        return not self._on

class Shoulder:
    def __init__(self, model, data, prefix="l_"):
        self._model = model
        self._data = data
        self.pitch = Joint(self._model, self._data, f"{prefix}shoulder_pitch")
        self.roll = Joint(self._model, self._data, f"{prefix}shoulder_roll")

    def _reset(self):
        self.pitch._reset()
        self.roll._reset()

    def _update(self):
        self.pitch._update()
        self.roll._update()

    def turn_on(self):
        self.pitch.turn_on()
        self.roll.turn_on()
        return True

    def turn_off(self):
        self.pitch.turn_off()
        self.roll.turn_off()
        return True

    def turn_off_smoothly(self):
        #TODO
        pass

    def is_on(self):
        return self.pitch.is_on() and self.roll.is_on()

    def is_off(self):
        return self.pitch.is_off() and self.roll.is_off()


class Elbow:
    def __init__(self, model, data, prefix="l_"):
        self._model = model
        self._data = data
        self.yaw = Joint(self._model, self._data, f"{prefix}elbow_yaw")
        self.pitch = Joint(self._model, self._data, f"{prefix}elbow_pitch")

    def _reset(self):
        self.pitch._reset()
        self.yaw._reset()

    def _update(self):
        self.yaw._update()
        self.pitch._update()


    def turn_on(self):
        self.pitch.turn_on()
        self.yaw.turn_on()
        return True

    def turn_off(self):
        self.pitch.turn_off()
        self.yaw.turn_off()
        return True

    def turn_off_smoothly(self):
        #TODO
        pass

    def is_on(self):
        return self.pitch.is_on() and self.yaw.is_on()

    def is_off(self):
        return self.pitch.is_off() and self.yaw.is_off()

class Wrist:
    def __init__(self, model, data, prefix="l_"):
        self._model = model
        self._data = data
        self.roll = Joint(self._model, self._data, f"{prefix}wrist_roll")
        self.pitch = Joint(self._model, self._data, f"{prefix}wrist_pitch")
        self.yaw = Joint(self._model, self._data, f"{prefix}wrist_yaw")

    def _reset(self):
        self.pitch._reset()
        self.roll._reset()
        self.yaw._reset()

    def _update(self):
        self.roll._update()
        self.pitch._update()
        self.yaw._update()


    def turn_on(self):
        self.roll.turn_on()
        self.pitch.turn_on()
        self.yaw.turn_on()
        return True

    def turn_off(self):
        self.roll.turn_off()
        self.pitch.turn_off()
        self.yaw.turn_off()
        return True

    def turn_off_smoothly(self):
        #TODO
        pass

    def is_on(self):
        return self.pitch.is_on() and self.yaw.is_on() and self.roll.is_on()


    def is_off(self):
        return self.pitch.is_off() and self.yaw.is_off() and self.roll.is_off()



class Gripper(Joint):
    def __init__(self, model, data, prefix="l_"):
        super().__init__(model, data, name=f"{prefix}hand_finger")
        self._limits = self._model.jnt_range[self._jnt_index]
        self.open()
        # self._limits = [-100, 100] # TODO fix

    def _reset(self):
        self.open() #FIXME?

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

    def _reset(self):
        self.pitch._reset()
        self.roll._reset()
        self.yaw._reset()

    def _update(self):
        self.roll._update()
        self.pitch._update()
        self.yaw._update()


    def turn_on(self):
        self.roll.turn_on()
        self.pitch.turn_on()
        self.yaw.turn_on()
        return True

    def turn_off(self):
        self.roll.turn_off()
        self.pitch.turn_off()
        self.yaw.turn_off()
        return True

    def turn_off_smoothly(self):
        #TODO
        pass

    def is_on(self):
        return self.pitch.is_on() and self.yaw.is_on() and self.roll.is_on()

    def is_off(self):
        return self.pitch.is_off() and self.yaw.is_off() and self.roll.is_off()



class Tripod(Joint):
    def __init__(self, model, data):
        super().__init__(model, data, name="tripod_joint")
        self._limits = self._model.jnt_range[self._jnt_index]
        #tripod_joint=0.0 => Torso=1.0
        #tripod_joint=0.2 =>Torso=1.2
        #We control the torso height
        self._goal_position=0.0
        self._present_position=0.0
        self.turn_on() #this is a fake fake joint

    def _reset(self):
        self._goal_position=self._data.ctrl[self._ctrl_index]
        self.goal_position=self._goal_position+1.0

    @property
    def height(self):
        self._present_position=self._data.qpos[self._qpos_index]
        return self._present_position+1.0

    def set_height(self, height):
        self.goal_position=height
        self._goal_position=self.goal_position-1.0

    def reset_height(self):
        self.goal_position=1.0
        self._goal_position=0.0


    def _update(self):
        np.clip(self.goal_position,1.0,1.2)
        self._goal_position=self.goal_position-1.0
        self._data.ctrl[self._ctrl_index] = self._goal_position
