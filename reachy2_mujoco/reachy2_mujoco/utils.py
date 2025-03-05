import mujoco
import numpy as np
from typing import Callable, Optional


def get_actuator_name(model, index: int) -> str:
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)


def get_actuator_index(model, name: str) -> int:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)

def get_joint_index(model, name: str) -> int:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)



def get_joint_name(model, index: int) -> str:
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)


def list_actuators(model):
    for i in range(20):
        name = get_actuator_name(model, i)
        print(i, name)

def get_mobile_base_add(model):

    floating_base_add = model.jnt_dofadr[
        np.where(model.jnt_type == 0)
    ]  # Assuming the mobile base freejoint is named "mobile_base"! the jnt_type==0 is a floating joint. 3 is a hinge

    mobile_base_add=[idx for idx in floating_base_add if get_joint_name(model, idx)=="mobile_base"]
    return mobile_base_add[0]

def get_mobile_base_qpos(model, data):
    add=get_mobile_base_add(model)
    return data.qpos[add:add+7]

def set_mobile_base_qpos(model, data, new_data):
    add=get_mobile_base_add(model)
    data.qpos[add:add+7]=new_data
    return data

def get_mobile_base_qvel(model, data):
    add=get_mobile_base_add(model)
    return data.qvel[add:add+6]


def set_mobile_base_qvel(model, data, new_data):
    add=get_mobile_base_add(model)
    data.qvel[add:add+6]=new_data
    return data


def get_wheels_qvel(model, data):

    r_vel=data.qvel[model.jnt_dofadr[get_joint_index(model, "right_wheel")]]
    b_vel=data.qvel[model.jnt_dofadr[get_joint_index(model, "back_wheel")]]
    l_vel=data.qvel[model.jnt_dofadr[get_joint_index(model, "left_wheel")]]
    return np.array([r_vel,b_vel,l_vel])

def get_wheels_qpos(model, data):

    r_pos=data.qpos[model.jnt_qposadr[get_joint_index(model, "right_wheel")]]
    b_pos=data.qpos[model.jnt_qposadr[get_joint_index(model, "back_wheel")]]
    l_pos=data.qpos[model.jnt_qposadr[get_joint_index(model, "left_wheel")]]

    return np.array([r_pos,b_pos,l_pos])




InterpolationFunc = Callable[[float], np.ndarray]

def minimum_jerk(
    starting_position: np.ndarray,
    goal_position: np.ndarray,
    duration: float,
    starting_velocity: Optional[np.ndarray] = None,
    starting_acceleration: Optional[np.ndarray] = None,
    final_velocity: Optional[np.ndarray] = None,
    final_acceleration: Optional[np.ndarray] = None,
) -> InterpolationFunc:
    """Compute the mimimum jerk interpolation function from starting position to goal position."""
    if starting_velocity is None:
        starting_velocity = np.zeros(starting_position.shape)
    if starting_acceleration is None:
        starting_acceleration = np.zeros(starting_position.shape)
    if final_velocity is None:
        final_velocity = np.zeros(goal_position.shape)
    if final_acceleration is None:
        final_acceleration = np.zeros(goal_position.shape)

    a0 = starting_position
    a1 = starting_velocity
    a2 = starting_acceleration / 2

    d1, d2, d3, d4, d5 = [duration**i for i in range(1, 6)]

    A = np.array(((d3, d4, d5), (3 * d2, 4 * d3, 5 * d4), (6 * d1, 12 * d2, 20 * d3)))
    B = np.array(
        (
            goal_position - a0 - (a1 * d1) - (a2 * d2),
            final_velocity - a1 - (2 * a2 * d1),
            final_acceleration - (2 * a2),
        )
    )
    X = np.linalg.solve(A, B)

    coeffs = [a0, a1, a2, X[0], X[1], X[2]]

    def f(t: float) -> np.ndarray:
        if t > duration:
            return goal_position
        return np.sum([c * t**i for i, c in enumerate(coeffs)], axis=0)

    return f
