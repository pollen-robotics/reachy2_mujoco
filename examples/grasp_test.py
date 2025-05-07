import numpy as np
import time
from FramesViewer.viewer import Viewer
import FramesViewer.utils as fv_utils


from reachy2_mujoco import ReachySDK

reachy = ReachySDK("localhost")


def get_T_torso_cube():
    cube_pos = np.around(np.array(reachy._data.body("goal").xpos), 2)
    # cube_mat = np.around(np.array(reachy._data.body("goal").xmat), 2)
    cube_mat = np.eye(3)
    trunk_pos = np.around(np.array(reachy._data.site("torso").xpos), 2)
    trunk_mat = np.around(np.array(reachy._data.site("torso").xmat), 2)

    T_world_cube = np.eye(4)
    T_world_cube[:3, :3] = cube_mat.reshape((3, 3))
    T_world_cube[:3, 3] = cube_pos

    T_world_torso = np.eye(4)
    T_world_torso[:3, :3] = trunk_mat.reshape((3, 3))
    T_world_torso[:3, 3] = trunk_pos

    T_torso_world = np.linalg.inv(T_world_torso)

    T_torso_cube = T_torso_world @ T_world_cube

    return T_torso_cube


# reachy.r_arm.gripper.close()
# reachy.r_arm.gripper.open()
# reachy.send_goal_positions()
# time.sleep(4)
reachy.l_arm.shoulder.roll.goal_position = 40
reachy.r_arm.shoulder.roll.goal_position = -30
reachy.r_arm.shoulder.pitch.goal_position = 40
reachy.r_arm.elbow.pitch.goal_position = -90
reachy.r_arm.elbow.yaw.goal_position = -30
reachy.r_arm.wrist.yaw.goal_position = -45
reachy.r_arm.wrist.pitch.goal_position = -45
reachy.r_arm.gripper.open()
reachy.send_goal_positions()
time.sleep(2)
# exit()

# fv = Viewer()
# fv.start()

T_torso_cube = get_T_torso_cube()
target_pose = T_torso_cube.copy()
target_pose = fv_utils.rotateInSelf(target_pose, [0, -90.0, 0], degrees=False)
target_pose = fv_utils.translateInSelf(target_pose, [0.05, 0.0, 0.0])
pregrasp_pose = target_pose.copy()
pregrasp_pose = fv_utils.translateInSelf(pregrasp_pose, [0, 0, 0.1])
# fv.pushFrame(T_torso_cube, "cube")
# fv.pushFrame(target_pose, "target")
# fv.pushFrame(pregrasp_pose, "pregrasp")

reachy.mobile_base.goto(-0.2, 0, 0)
reachy.r_arm.goto(pregrasp_pose)
time.sleep(2)
reachy.mobile_base.goto(0.0, 0, 0)
time.sleep(1)
reachy.r_arm.goto(target_pose)
time.sleep(2)
reachy.r_arm.gripper.close()
time.sleep(2)
lift_pose = target_pose.copy()
lift_pose = fv_utils.translateAbsolute(lift_pose, [0, 0, 0.1])
reachy.r_arm.goto(lift_pose)
