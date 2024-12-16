import yaml
import reachy2_sdk_api.orbita2d_pb2 as orbita2d_pb2
from google.protobuf.timestamp_pb2 import Timestamp


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


def get_current_timestamp(bridge_node) -> Timestamp:
    t = Timestamp()
    t.FromNanoseconds(int(bridge_node.get_clock()))

    return t


def cleanup_fields(FieldEnum, fields):
    if FieldEnum.NONE in fields:
        cleanup_fields = []
    elif FieldEnum.ALL in fields:
        cleanup_fields = list(FieldEnum.keys())
        cleanup_fields.remove("ALL")
        cleanup_fields.remove("NONE")
    else:
        cleanup_fields = [
            FieldEnum.DESCRIPTOR.values_by_number[field].name for field in fields
        ]

    cleanup_fields = [f.lower() for f in cleanup_fields]

    return cleanup_fields


def extract_fields(FieldEnum, fields, conversion_table, component) -> dict:
    grpc_state = {}

    for f in cleanup_fields(FieldEnum, fields):
        if f not in conversion_table:
            continue
        grpc_state[f] = conversion_table[f](component)

    return grpc_state
