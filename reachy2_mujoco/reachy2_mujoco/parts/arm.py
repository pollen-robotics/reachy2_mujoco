from reachy2_mujoco.parts.joints import Shoulder, Elbow, Wrist, Gripper
from reachy2_symbolic_ik.control_ik import ControlIK
import os


class Arm:
    def __init__(self, model, data, prefix="l_"):
        self._model = model
        self._data = data
        self._prefix = prefix
        self._urdf_path = "/".join(os.path.realpath(__file__).split("/")[:-3]) + "/description/modified_urdf/reachy2.urdf"
        self._control_ik = ControlIK(urdf_path=self._urdf_path)

        # self._control_ik.symbolic_ik_solver

        self.shoulder = Shoulder(self._model, self._data, prefix=prefix)
        self.elbow = Elbow(self._model, self._data, prefix=prefix)
        self.wrist = Wrist(self._model, self._data, prefix=prefix)

        self.gripper = Gripper(self._model, self._data, prefix=prefix)

    def _update(self):
        self.shoulder._update()
        self.elbow._update()
        self.wrist._update()
        self.gripper._update()

    def get_current_positions(self):
        return [
            self.shoulder.pitch.current_position,
            self.shoulder.roll.current_position,
            self.elbow.yaw.current_position,
            self.elbow.pitch.current_position,
            self.wrist.roll.current_position,
            self.wrist.pitch.current_position,
            self.wrist.yaw.current_position
        ]

    # TODO implement
    def goto(self, target, duration=2):
        """
        Joint space goto
        target : list of floats OR 4x4 homogeneous matrix
            list : [shoulder_pitch, shoulder_roll, elbow_yaw, elbow_pitch, wrist_roll, wrist_pitch, wrist_yaw]
            matrix : 4x4 matrix representing the target pose
        """
        # TODO implement duration

        if target.shape == (4, 4):
            # Compute IK etc
            # target = self._control_ik.symbolic_ik_solver(target)
            # 
            # target = self._control_ik.symbolic_inverse_kinematics(name=self._prefix)
            pass
        elif len(target) == 7:
            self.shoulder.pitch.goal_position = target[0]
            self.shoulder.roll.goal_position = target[1]
            self.elbow.yaw.goal_position = target[2]
            self.elbow.pitch.goal_position = target[3]
            self.wrist.roll.goal_position = target[4]
            self.wrist.pitch.goal_position = target[5]
            self.wrist.yaw.goal_position = target[6]
        else:
            raise ValueError("Invalid target shape")


