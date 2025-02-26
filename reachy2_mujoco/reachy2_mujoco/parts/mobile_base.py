import pyquaternion
import numpy as np
import reachy2_mujoco.utils as utils

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
        self._pid = [0.25, 0, 0]

        self.position = np.zeros(3)  # x, y, theta

    def _update_position(self):
        # print(f"DEBUG: add {utils.get_mobile_base_add(self._model)}")
        # print(f"DEBUG: qpos {utils.get_mobile_base_qpos(self._model,self._data)}")
        # print(f"DEBUG: qvel {utils.get_mobile_base_qvel(self._model,self._data)}")

        mobile_base_qpos=utils.get_mobile_base_qpos(self._model,self._data)
        # self.position[:2] = self._data.qpos[:2] - self._pos_offset[:2]
        self.position[:2] = mobile_base_qpos[:2] - self._pos_offset[:2]

        # quat = self._data.qpos[3:7]
        quat = mobile_base_qpos[3:7]
        yaw = pyquaternion.Quaternion(quat).yaw_pitch_roll[0]

        self.position[2] = yaw - self._pos_offset[2]

    def _update(self):
        self._update_position()
        mobile_base_qvel=utils.get_mobile_base_qvel(self._model,self._data)
        vector_2d = np.array(self._target_position[:2] - self.position[:2]) * self._pid[0]
        # self._data.qvel[:2] = vector_2d
        mobile_base_qvel[:2] = vector_2d
        # theta
        # self._data.qvel[5] = self._target_position[2] - self.position[2] * self._pid[0]
        mobile_base_qvel[5] = np.array(self._target_position[2] - self.position[2]) * self._pid[0]
        utils.set_mobile_base_qvel(self._model,self._data,mobile_base_qvel)

    # TODO not working
    def reset_odometry(self):
        # self._pos_offset = self.position
        print("Not implemented")
        pass

    # TODO implement better (with PID ?)
    def goto(self, x, y, theta):
        self._target_position = np.array([x, y, theta])

    # TODO match real behvior (command for only 200ms ? )
    def set_goal_speed(self, vx, vy, vtheta):
        mobile_base_qvel=utils.get_mobile_base_qvel(self._model,self._data)
        mobile_base_qvel[:3] = [vx, vy, vtheta]
        utils.set_mobile_base_qvel(self._model,self._data,mobile_base_qvel)
