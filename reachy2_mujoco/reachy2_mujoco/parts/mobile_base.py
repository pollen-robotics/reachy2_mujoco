import pyquaternion
import numpy as np
import reachy2_mujoco.utils as utils
import time

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
        self._pid_translation = [2.5, 0.5, 0.5]
        self._pid_rotation = [3, 1.0, 1.0]

        self.position = np.zeros(3)  # x, y, theta
        self.velocity = np.zeros(3)  # x, y, theta

        self.left_idx=utils.get_actuator_index(self._model,"left_wheel")
        self.right_idx=utils.get_actuator_index(self._model,"right_wheel")
        self.back_idx=utils.get_actuator_index(self._model,"back_wheel")
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

        # self.ctrl_mat=1.0/self.wheel_rad*np.matrix([
        #     [-self.wheel_base_rad, 1.0, 0.0],
        #     [-self.wheel_base_rad, 0.5, -np.sin(np.pi/3.0)],
        #     [-self.wheel_base_rad, 0.5, -np.sin(np.pi/3.0)]]
        # )

        self.ctrl_mat=1.0/self.wheel_rad*np.matrix([
            [-np.sin(np.pi/3.0), np.cos(np.pi/3.0), self.wheel_base_rad], #left
            [0, -1, self.wheel_base_rad], #back
            [np.sin(np.pi/3.0), np.cos(np.pi/3.0), self.wheel_base_rad], #right
         ]
        )
        # self.ctrl_mat=1.0/self.wheel_rad*np.matrix([
        #     [np.cos(2*np.pi/3.0), -np.sin(2*np.pi/3.0),self.wheel_base_rad],
        #     [-1, 0.0, self.wheel_base_rad],
        #     [np.cos(2*np.pi/3.0), -np.sin(2*np.pi/3.0),self.wheel_base_rad]]
        # )


    def _update_position(self):

        mobile_base_qpos=utils.get_mobile_base_qpos(self._model,self._data)
        # self.position[:2] = self._data.qpos[:2] - self._pos_offset[:2]
        self.position[:2] = mobile_base_qpos[:2] - self._pos_offset[:2]

        # quat = self._data.qpos[3:7]
        quat = mobile_base_qpos[3:7]
        yaw = pyquaternion.Quaternion(quat).yaw_pitch_roll[0]

        self.position[2] = yaw - self._pos_offset[2]
        qvel=utils.get_mobile_base_qvel(self._model,self._data)
        self.velocity=np.zeros(3)
        self.velocity[:2]=qvel[:2]
        self.velocity[2]=qvel[5]

    def _update(self):
        dt=time.time()-self._prevt
        self._prevt=time.time()
        self._update_position()

        # mobile_base_qvel=utils.get_mobile_base_qvel(self._model,self._data)
        # vector_2d = np.clip(np.array(self._target_position[:2] - self.position[:2]) * self._pid[0],-0.1,0.1 )
        # # self._data.qvel[:2] = vector_2d
        # mobile_base_qvel[:2] = vector_2d
        # mobile_base_qvel[2] = 0.0

        # # theta
        # # self._data.qvel[5] = self._target_position[2] - self.position[2] * self._pid[0]
        # mobile_base_qvel[5] = np.clip(np.array(self._target_position[2] - self.position[2]) * self._pid[0], -0.1,0.1)
        # mobile_base_qvel[3] = 0.0
        # mobile_base_qvel[4] = 0.0

        # utils.set_mobile_base_qvel(self._model,self._data,mobile_base_qvel)


        # in velocity
        # theta_goal_vel=0
        # #theta
        # theta_err=(self._target_position[2] - self.position[2])
        # theta_goal_vel=  theta_err/dt

        # #vx,vy
        # xy_goal_vel=np.zeros(2)
        # xyerr=np.array(self._target_position[:2] - self.position[:2])
        # xy_goal_vel = xyerr/dt

        # xyvel_err=xy_goal_vel-np.array(self.velocity[:2])
        # thetavel_err=theta_goal_vel-self.velocity[2]

        # goalvec=np.zeros(3)

        # if np.abs(thetavel_err)>0.02:
        #     goalvec[2]=np.clip(thetavel_err*self._pid_rotation[0],-5.0,5.0)
        # if np.abs(xyvel_err).any()>0.05:
        #     goalvec[:2]=np.clip(xyvel_err*self._pid_translation[0],-6.0,6.0)


        #position PID
        goalvec=np.zeros(3)

        pos_err=np.round(self._target_position[:2]-self.position[:2],3)
        if np.abs(pos_err).all()<0.02:
            pos_err=np.zeros(2)
        d_pos_err = np.clip(np.round((pos_err - self.d_position_errors) / dt,3), -1.0,1.0)
        i_err = np.clip(self.acc_position_errors + pos_err, -self.MAX_POS_ACC_ERR, self.MAX_POS_ACC_ERR)

        pos_goal = pos_err * self._pid_translation[0] + i_err *self._pid_translation[1] + d_pos_err * self._pid_translation[2]
        self.d_position_errors = d_pos_err
        self.acc_position_errors = i_err
        goalvec[:2]=pos_goal


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



        ctrlvec=self.ctrl_mat.dot(goalvec)
        ctrlvec=ctrlvec.reshape((3,1))

        # print(f"DEBUG dt: {dt} pos: {self.position}\ntarget_pos: {self._target_position}\npos_err: {pos_err}\nd_pos_err: {d_pos_err}\nd_position_errors: {self.d_position_errors}\ngoalvec: {goalvec}\nctrlvec: {ctrlvec}\nmat: {self.ctrl_mat}")
         #print(f"DEBUG pos: {self.position}\ntheta_vel_err: {thetavel_err}\nxy_vel_err: {xyvel_err}\ntheta_err: {theta_err}\nxy_err: {xyerr}\ngoalvec: {goalvec}\nctrlvec: {ctrlvec}\nmat: {self.ctrl_mat}")
        # ctrlvec=np.clip(ctrlvec, -30.0,30.0)
        self._data.ctrl[utils.get_actuator_index(self._model, "left_wheel")]=ctrlvec[0]
        self._data.ctrl[utils.get_actuator_index(self._model, "back_wheel")]=ctrlvec[1]
        self._data.ctrl[utils.get_actuator_index(self._model, "right_wheel")]=ctrlvec[2]


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
        mobile_base_qvel = [vx, vy, 0.0, 0.0, 0.0, vtheta]
        utils.set_mobile_base_qvel(self._model,self._data,mobile_base_qvel)
