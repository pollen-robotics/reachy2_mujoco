import pyquaternion
import numpy as np
from ..utils import get_actuator_index, get_mobile_base_qpos, get_mobile_base_qvel, get_wheels_qpos, get_wheels_qvel
import time

import logging
logger = logging.getLogger(__name__)
logging.basicConfig()
logger.setLevel(logging.INFO)


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
        self._pid_translation = [2, 0, 0]
        self._pid_rotation = [10, 5, 0.0]

        self.position = np.zeros(3)  # x, y, theta
        self.velocity = np.zeros(3)  # x, y, theta
        self.wheel_qvel = np.zeros(3) #left, back, right
        self.wheel_qpos = np.zeros(3) #left, back, right
        self.left_idx=get_actuator_index(self._model,"left_wheel")
        self.right_idx=get_actuator_index(self._model,"right_wheel")
        self.back_idx=get_actuator_index(self._model,"back_wheel")
        self.wheel_rad=0.105
        self.wheel_base_rad=0.198
        self._prevt=time.time()
        self.d_position_errors = np.zeros(2)
        self.acc_position_errors = np.zeros(2)
        self.acc_orientation_errors = 0
        self.d_orientation_errors = 0
        self.acc_position_errors = 0
        self.MAX_POS_ACC_ERR=0.1
        self.MAX_ORI_ACC_ERR=0.1

        self.ctrl_mat=1.0/self.wheel_rad*np.matrix([
            [-np.sin(np.pi/3.0), np.cos(np.pi/3.0), self.wheel_base_rad], #left
            [0, -1, self.wheel_base_rad], #back
            [np.sin(np.pi/3.0), np.cos(np.pi/3.0), self.wheel_base_rad], #right
         ]
        )

    def _reset(self):
        self._target_position=np.zeros(3) #TODO reset to key

    def _update_position(self):

        mobile_base_qpos=get_mobile_base_qpos(self._model,self._data)
        # self.position[:2] = self._data.qpos[:2] - self._pos_offset[:2]
        self.position[:2] = mobile_base_qpos[:2] - self._pos_offset[:2]

        # quat = self._data.qpos[3:7]
        quat = mobile_base_qpos[3:7]
        yaw = pyquaternion.Quaternion(quat).yaw_pitch_roll[0]

        self.position[2] = yaw - self._pos_offset[2]
        qvel=get_mobile_base_qvel(self._model,self._data)
        self.velocity=np.zeros(3)
        self.velocity[:2]=qvel[:2]
        self.velocity[2]=qvel[5]

        self.wheel_qvel=get_wheels_qvel(self._model, self._data)
        self.wheel_qpos=get_wheels_qpos(self._model, self._data)

    def _update(self):
        dt=time.time()-self._prevt
        self._prevt=time.time()
        self._update_position()


        #position PID
        goalvec=np.zeros(3)

        # pos_err=np.round(self._target_position[:2]-self.position[:2],3)
        pos_err=np.round(self.position[:2]-self._target_position[:2],3)
        dist_err=np.linalg.norm(pos_err)
        if np.abs(dist_err).all()<0.02:
            goalvec[:2]=np.zeros(2)

        else:

            xy_mat=np.matrix([
                [-np.cos(self.position[2]),-np.sin(self.position[2])],
                [np.sin(self.position[2]),-np.cos(self.position[2])],
            ])

            xy_cmd=xy_mat.dot(pos_err)
            xy_cmd/=dist_err

            # d_pos_err = np.clip(np.round((pos_err - self.d_position_errors) / dt,3), -1.0,1.0)

            # i_err = np.clip(self.acc_position_errors + pos_err, -self.MAX_POS_ACC_ERR, self.MAX_POS_ACC_ERR)

            # pos_goal = pos_err * self._pid_translation[0] + i_err *self._pid_translation[1] + d_pos_err * self._pid_translation[2]
            # self.d_position_errors = d_pos_err
            # self.acc_position_errors = i_err

            goalvec[:2]=xy_cmd*self._pid_translation[0]


        #orientation pid

        ori_err=np.round(self._target_position[2]-self.position[2],3)
        if np.abs(ori_err)<0.02:
            ori_err=0.0
        d_ori_err = np.clip(np.round((ori_err - self.d_orientation_errors) / dt,3), -1.0,1.0)
        i_err = np.clip(self.acc_orientation_errors + ori_err, -self.MAX_ORI_ACC_ERR, self.MAX_ORI_ACC_ERR)

        ori_goal = ori_err * self._pid_rotation[0] + i_err *self._pid_rotation[1] + d_ori_err * self._pid_rotation[2]
        self.d_orientation_errors = d_ori_err
        self.acc_orientation_errors = i_err

        goalvec[2]=ori_goal


        #velocity pid
        # goalvec=np.zeros(3)
        # goalvec[:]=self._target_position[:]

        ctrlvec=self.ctrl_mat.dot(goalvec)
        ctrlvec=ctrlvec.reshape((3,1))

        # print(f'DEBUG: target: {self._target_position}\nctrl: {ctrlvec.T}\npos: {self.position}\nvel: {self.velocity}\nw_vel: {self.wheel_qvel}\nw_pos: {self.wheel_qpos}\npos_err: {pos_err}\ndist_err: {dist_err}')

        self._data.ctrl[get_actuator_index(self._model, "left_wheel")]=ctrlvec[0]
        self._data.ctrl[get_actuator_index(self._model, "back_wheel")]=ctrlvec[1]
        self._data.ctrl[get_actuator_index(self._model, "right_wheel")]=ctrlvec[2]


    # TODO not working
    def reset_odometry(self):
        # self._pos_offset = self.position
        logger.warning("Not implemented")
        pass

    # TODO implement better (with PID ?)
    def goto(self, x, y, theta):
        self._target_position = np.array([x, y, np.radians(theta)])

    # TODO match real behvior (command for only 200ms ? )
    def set_goal_speed(self, vx, vy, vtheta):
        mobile_base_qvel=get_mobile_base_qvel(self._model,self._data)
        mobile_base_qvel = [vx, vy, 0.0, 0.0, 0.0, vtheta]
        set_mobile_base_qvel(self._model,self._data,mobile_base_qvel)

    @property
    def odometry(self):
        return {'x': self.position[0], 'y': self.position[1], 'theta': np.degrees(self.position[2])}
