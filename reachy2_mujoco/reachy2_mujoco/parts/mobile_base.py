import pyquaternion
import numpy as np


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
        self._pid = [1.3, 0, 0]

        self.position = np.zeros(3)  # x, y, theta

    def _update_position(self):
        self.position[:2] = self._data.qpos[:2] - self._pos_offset[:2]

        quat = self._data.qpos[3:7]
        yaw = pyquaternion.Quaternion(quat).yaw_pitch_roll[0]

        self.position[2] = yaw - self._pos_offset[2]

    def _update(self):
        self._update_position()

        vector_2d = np.array(self._target_position[:2]) - self.position[:2] * self._pid[0]
        self._data.qvel[:2] = vector_2d

        # theta
        self._data.qvel[5] = self._target_position[2] - self.position[2] * self._pid[0]

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
        self._data.qvel[:3] = [vx, vy, vtheta]
