from reachy2_mujoco.parts.joints import Shoulder, Elbow, Wrist, Gripper
from reachy2_symbolic_ik.control_ik import ControlIK
import os
import numpy as np


class Arm:
    def __init__(self, model, data, control_ik, prefix="l_"):
        self._model = model
        self._data = data
        self._prefix = prefix
        self._urdf_path = "/".join(os.path.realpath(__file__).split("/")[:-3]) + "/description/modified_urdf/reachy2.urdf"
        self._control_ik: ControlIK = control_ik

        self.shoulder = Shoulder(self._model, self._data, prefix=prefix)
        self.elbow = Elbow(self._model, self._data, prefix=prefix)
        self.wrist = Wrist(self._model, self._data, prefix=prefix)

        self.gripper = Gripper(self._model, self._data, prefix=prefix)

    def _update(self):
        self.shoulder._update()
        self.elbow._update()
        self.wrist._update()
        self.gripper._update()

    def get_present_positions(self):
        return [
            self.shoulder.pitch.present_position,
            self.shoulder.roll.present_position,
            self.elbow.yaw.present_position,
            self.elbow.pitch.present_position,
            self.wrist.roll.present_position,
            self.wrist.pitch.present_position,
            self.wrist.yaw.present_position,
        ]

    def set_goal_positions(self, target):
        self.shoulder.pitch.goal_position = target[0]
        self.shoulder.roll.goal_position = target[1]
        self.elbow.yaw.goal_position = target[2]
        self.elbow.pitch.goal_position = target[3]
        self.wrist.roll.goal_position = target[4]
        self.wrist.pitch.goal_position = target[5]
        self.wrist.yaw.goal_position = target[6]

    # TODO implement
    def goto(self, target, duration=2):
        """
        Joint space goto
        target : list of floats OR 4x4 homogeneous matrix
            list : [shoulder_pitch, shoulder_roll, elbow_yaw, elbow_pitch, wrist_roll, wrist_pitch, wrist_yaw]
            matrix : 4x4 matrix representing the target pose
        """
        # TODO implement duration

        target = np.array(target)

        if target.shape == (4, 4):
            # Compute IK etc
            # target = self._control_ik.symbolic_ik_solver(target)
            #
            # target = self._control_ik.symbolic_inverse_kinematics(name=self._prefix)
            sol, is_reachable, state = self._control_ik.symbolic_inverse_kinematics(
                f"{self._prefix}arm",
                target,
                "continuous",
                current_joints=self.get_present_positions(),
                constrained_mode="unconstrained",
                # current_pose=current_pose,
                d_theta_max=0.02,
            )
            if not is_reachable:
                print("Target is not reachable")
                print(state)
                return

            self.set_goal_positions(sol)

        elif len(target) == 7:
            self.set_goal_positions(target)
        else:
            raise ValueError("Invalid target shape")
