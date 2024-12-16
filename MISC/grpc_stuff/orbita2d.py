import grpc
from collections import namedtuple
import math
import reachy2_sdk_api.orbita2d_pb2_grpc as orbita2d_pb2_grpc
from components import Component
from google.protobuf.wrappers_pb2 import BoolValue, FloatValue
from reachy2_sdk_api.component_pb2 import ComponentId, JointLimits, PIDGains
from reachy2_sdk_api.orbita2d_pb2 import (
    Float2d,
    Limits2d,
    ListOfOrbita2d,
    Orbita2d,
    Orbita2dCommand,
    Orbita2dField,
    Orbita2dsCommand,
    Orbita2dState,
    Orbita2dStateRequest,
    Orbita2dStatus,
    Orbita2dStreamStateRequest,
    PID2d,
    Pose2d,
    Vector2d,
)
from reachy2_sdk_api.component_pb2 import ComponentId
from utils import axis_from_str, extract_fields, get_current_timestamp

Orbita2dComponents = namedtuple("Orbita2dComponents", ["actuator", "axis1", "axis2", "raw_motor_1", "raw_motor_2"])


class Orbita2dServicer(orbita2d_pb2_grpc.Orbita2dServiceServicer):
    default_fields = [
        Orbita2dField.PRESENT_POSITION,
        Orbita2dField.GOAL_POSITION,
        Orbita2dField.PRESENT_SPEED,
        Orbita2dField.PRESENT_LOAD,
        Orbita2dField.TEMPERATURE,
        Orbita2dField.JOINT_LIMITS,
        Orbita2dField.TORQUE_LIMIT,
        Orbita2dField.SPEED_LIMIT,
        Orbita2dField.PID,
        Orbita2dField.COMPLIANT,
    ]

    def __init__(self, bridge_node):
        self.bridge_node = bridge_node

    @classmethod
    def get_info(cls, orbita3d: Component) -> Orbita2d:
        return Orbita2d(
            id=ComponentId(
                id=orbita3d.id,
                name=orbita3d.name,
            ),
        )

    # Setup utils
    def get_orbita2d_components(self, component_id: ComponentId, context: grpc.ServicerContext) -> Orbita2dComponents:
        if not hasattr(self, "_lazy_components"):
            self._lazy_components = {}

        components = self.bridge_node.components

        c = components.get_by_component_id(component_id)
        if c is None:
            context.abort(
                grpc.StatusCode.NOT_FOUND,
                f"Could not find component with id '{component_id}'.",
            )

        if c.type != "orbita2d":
            context.abort(
                grpc.StatusCode.INVALID_ARGUMENT,
                f"Component '{component_id}' is not an orbita2d.",
            )

        if c.id not in self._lazy_components:
            orbita2d = components.get_by_component_id(component_id)
            orbita2d_axis1 = components.get_by_name(f"{orbita2d.name}_{orbita2d.extra['axis1']}")
            orbita2d_axis2 = components.get_by_name(f"{orbita2d.name}_{orbita2d.extra['axis2']}")
            orbita2d_raw_motor_1 = components.get_by_name(f"{orbita2d.name}_raw_motor_1")
            orbita2d_raw_motor_2 = components.get_by_name(f"{orbita2d.name}_raw_motor_2")

            self._lazy_components[c.id] = Orbita2dComponents(
                orbita2d,
                orbita2d_axis1,
                orbita2d_axis2,
                orbita2d_raw_motor_1,
                orbita2d_raw_motor_2,
            )

        return self._lazy_components[c.id]

    def GetAllOrbita2d(self, request, context):
        # TODO
        return super().GetAllOrbita2d(request, context)

    def GetState(self, request, context):
        orbita2d_components = self.get_orbita2d_components(request.id, context=context)
        state = extract_fields(Orbita2dField, request.fields, conversion_table, orbita2d_components)

        state["timestamp"] = get_current_timestamp(self.bridge_node)
        state["temperature"] = Float2d(motor_1=FloatValue(value=40.0), motor_2=FloatValue(value=40.0))
        state["joint_limits"] = Limits2d(
            axis_1=JointLimits(min=FloatValue(value=0.0), max=FloatValue(value=100.0)),
            axis_2=JointLimits(min=FloatValue(value=0.0), max=FloatValue(value=100.0)),
        )
        return Orbita2dState(**state)


conversion_table = {
    "id": lambda o: ComponentId(id=o.actuator.id, name=o.actuator.name),
    "present_position": lambda o: Pose2d(
        axis_1=FloatValue(value=o.axis1.state["position"]),
        axis_2=FloatValue(value=o.axis2.state["position"]),
    ),
    "present_speed": lambda o: Vector2d(
        x=FloatValue(value=o.axis1.state["velocity"]),
        y=FloatValue(value=o.axis2.state["velocity"]),
    ),
    "present_load": lambda o: Vector2d(
        x=FloatValue(value=o.axis1.state["effort"]),
        y=FloatValue(value=o.axis2.state["effort"]),
    ),
    "compliant": lambda o: BoolValue(value=not o.actuator.state["torque"]),
    "goal_position": lambda o: Pose2d(
        axis_1=FloatValue(value=o.axis1.state["target_position"]),
        axis_2=FloatValue(value=o.axis2.state["target_position"]),
    ),
    "speed_limit": lambda o: Float2d(
        motor_1=(
            FloatValue(value=o.raw_motor_1.state["speed_limit"])
            if not math.isnan(o.raw_motor_1.state["speed_limit"])
            else FloatValue(value=100.0)
        ),
        motor_2=(
            FloatValue(value=o.raw_motor_2.state["speed_limit"])
            if not math.isnan(o.raw_motor_2.state["speed_limit"])
            else FloatValue(value=100.0)
        ),
    ),
    "torque_limit": lambda o: Float2d(
        motor_1=(
            FloatValue(value=o.raw_motor_1.state["torque_limit"])
            if not math.isnan(o.raw_motor_1.state["torque_limit"])
            else FloatValue(value=100.0)
        ),
        motor_2=(
            FloatValue(value=o.raw_motor_2.state["torque_limit"])
            if not math.isnan(o.raw_motor_2.state["torque_limit"])
            else FloatValue(value=100.0)
        ),
    ),
    "pid": lambda o: PID2d(
        motor_1=PIDGains(
            p=(
                FloatValue(value=o.raw_motor_1.state["p_gain"])
                if not math.isnan(o.raw_motor_1.state["p_gain"])
                else FloatValue(value=100.0)
            ),
            i=(
                FloatValue(value=o.raw_motor_1.state["i_gain"])
                if not math.isnan(o.raw_motor_1.state["i_gain"])
                else FloatValue(value=100.0)
            ),
            d=(
                FloatValue(value=o.raw_motor_1.state["d_gain"])
                if not math.isnan(o.raw_motor_1.state["d_gain"])
                else FloatValue(value=100.0)
            ),
        ),
        motor_2=PIDGains(
            p=(
                FloatValue(value=o.raw_motor_2.state["p_gain"])
                if not math.isnan(o.raw_motor_2.state["p_gain"])
                else FloatValue(value=100.0)
            ),
            i=(
                FloatValue(value=o.raw_motor_2.state["i_gain"])
                if not math.isnan(o.raw_motor_2.state["i_gain"])
                else FloatValue(value=100.0)
            ),
            d=(
                FloatValue(value=o.raw_motor_2.state["d_gain"])
                if not math.isnan(o.raw_motor_2.state["d_gain"])
                else FloatValue(value=100.0)
            ),
        ),
    ),
}
