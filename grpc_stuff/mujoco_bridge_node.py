from components import ComponentsHolder
from parts import PartsHolder
from utils import parse_reachy_config
import time


class MujocoBridgeNode:
    def __init__(self, config):
        self.config = parse_reachy_config(config)
        self.components = ComponentsHolder(self.config)
        self.joint_names = [
            "l_elbow_yaw",
            "l_shoulder_pitch",
            "l_elbow_pitch",
            "l_wrist_yaw",
            "l_shoulder",
            "l_shoulder_raw_motor_2",
            "l_elbow",
            "l_elbow_raw_motor_2",
            "r_wrist_raw_motor_1",
            "r_wrist_raw_motor_2",
            "neck_pitch",
            "l_shoulder_roll",
            "l_hand_finger",
            "r_wrist_raw_motor_3",
            "l_wrist_raw_motor_1",
            "r_wrist",
            "l_wrist_pitch",
            "l_shoulder_raw_motor_1",
            "neck_roll",
            "l_wrist_roll",
            "neck_yaw",
            "neck",
            "r_shoulder_roll",
            "l_wrist_raw_motor_3",
            "r_elbow_raw_motor_2",
            "neck_raw_motor_1",
            "l_wrist_raw_motor_2",
            "r_hand",
            "r_shoulder_raw_motor_2",
            "r_wrist_yaw",
            "l_wrist",
            "r_wrist_pitch",
            "neck_raw_motor_2",
            "r_shoulder_raw_motor_1",
            "neck_raw_motor_3",
            "r_elbow_raw_motor_1",
            "r_hand_raw_motor_1",
            "r_elbow",
            "r_shoulder",
            "l_hand",
            "r_wrist_roll",
            "r_hand_finger",
            "r_elbow_pitch",
            "r_elbow_yaw",
            "r_shoulder_pitch",
            "l_elbow_raw_motor_1",
            "l_hand_raw_motor_1",
        ]

        for name in self.joint_names:
            self.components.add_component(name)

        self.parts = PartsHolder(self.config, self.components)

    def get_clock(self):
        t_seconds = time.time()
        t_nanoseconds = t_seconds * 1e9
        return t_nanoseconds


if __name__ == "__main__":
    mbn = MujocoBridgeNode("reachy.yaml")
