import yaml
import reachy2_sdk_api.orbita2d_pb2 as orbita2d_pb2


def parse_reachy_config(reachy_config_path: str) -> dict:
    with open(reachy_config_path, "r") as f:
        config = yaml.safe_load(f)
    return config


def axis_from_str(name: str) -> orbita2d_pb2.Axis:
    if name == "roll":
        return orbita2d_pb2.Axis.ROLL
    elif name == "pitch":
        return orbita2d_pb2.Axis.PITCH
    elif name == "yaw":
        return orbita2d_pb2.Axis.YAW
    else:
        raise ValueError(f"Unknown axis '{name}'.")
