import mujoco


def get_actuator_name(model, index: int) -> str:
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)


def get_actuator_index(model, name: str) -> int:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)


def list_actuators(model):
    for i in range(20):
        name = get_actuator_name(model, i)
        print(i, name)
