import numpy as np
import time
from FramesViewer.viewer import Viewer
import FramesViewer.utils as fv_utils
from reachy2_mujoco import ReachySDK
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--kp", type=float, default=1)
parser.add_argument("--kd", type=float, default=1)
args = parser.parse_args()

reachy = ReachySDK("localhost")


def get_body_pos(body_name):
    return reachy._data.body(body_name).xpos


def get_body_vel(body_name):
    return reachy._data.body(body_name).cvel


def get_body_mat(body_name):
    return reachy._data.body(body_name).xmat


def get_wrist_plane_center_dist():
    l_wrist_link_pos = np.array(get_body_pos("l_wrist_link"))
    plane_center_pos = np.array(reachy._data.geom("plane_center").xpos)

    return np.sqrt(np.sum((l_wrist_link_pos - plane_center_pos) ** 2))


wrist_plane_dist = get_wrist_plane_center_dist()
current_tip = np.array(reachy.l_arm._control_ik.symbolic_ik_solver["l_arm"].tip_position)
new_tip = [0, 0, wrist_plane_dist]
reachy.l_arm._control_ik.symbolic_ik_solver["l_arm"].tip_position[2] = new_tip[2]

T_torso_hand = np.eye(4)
T_torso_hand = fv_utils.rotateInSelf(T_torso_hand, [0, -90, 0], degrees=True)
T_torso_hand[:3, 3] = [0.6, 0.27, -0.18]

reachy.l_arm.goto(T_torso_hand)
time.sleep(2)


def reset_ball():
    # racket_pos = get_body_pos("racket")
    plane_center_pos = reachy._data.geom("plane_center").xpos
    ball_pos = plane_center_pos.copy() + [0.0, 0.0, 0.05]
    reachy._data.qpos[-7:-4] = ball_pos
    reachy._data.qvel[-6:] = 0


reset_ball()
# print(get_body_pos("ball"))
# exit()


def apply_target(target):
    sol, is_reachable, _ = reachy.l_arm.get_ik_sol(target)
    if not is_reachable:
        print("Target is not reachable")
        return

    reachy.l_arm.set_goal_positions(np.rad2deg(sol))


def P_control(kp, error):
    control = kp * error
    return np.array(control)


def PD_control(kp, kd, error, vel):
    control = kp * error + kd * vel
    return np.array(control)


# start control
T_torso_hand_ref = T_torso_hand.copy()
s = time.time()
while True:

    # print(get_tip_offset())
    # s_target = 10 * np.sin(2 * np.pi * 0.5 * time.time())
    # T_torso_hand = fv_utils.rotateInSelf(
    #     T_torso_hand_ref,
    #     [0, s_target, 0],
    #     degrees=True,
    # )

    # print(get_tip_offset())
    plane_center_pos = reachy._data.geom("plane_center").xpos[:2]
    ball_pos = get_body_pos("ball")[:2]
    ball_vel = get_body_vel("ball")[:2]

    # offset = [5, 0]
    error = plane_center_pos - ball_pos
    print(error)
    # error = [0, 0]
    control = PD_control(args.kp, args.kd, error, ball_vel)
    control = np.clip(control, -20, 20)
    T_torso_hand = fv_utils.rotateInSelf(
        T_torso_hand_ref,
        # [0, 0, -control[1]],
        [0, control[0], control[1]],
        degrees=True,
    )
    reachy.l_arm.set_target(T_torso_hand)

    if time.time() - s > 10:
        reset_ball()
        s = time.time()

    time.sleep(0.01)
