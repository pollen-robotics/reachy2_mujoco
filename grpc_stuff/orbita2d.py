import reachy2_sdk_api.orbita2d_pb2_grpc as orbita2d_pb2_grpc
from components import Component
from reachy2_sdk_api.orbita2d_pb2 import Orbita2d
from reachy2_sdk_api.component_pb2 import ComponentId
from utils import axis_from_str


class Orbita2dServicer(orbita2d_pb2_grpc.Orbita2dServiceServicer):
    def __init__(self, bridge_node):
        self.bridge_node = bridge_node

    @classmethod
    def get_info(cls, orbita2d: Component) -> Orbita2d:
        return Orbita2d(
            id=ComponentId(
                id=orbita2d.id,
                name=orbita2d.name,
            ),
            axis_1=axis_from_str(orbita2d.extra["axis1"]),
            axis_2=axis_from_str(orbita2d.extra["axis2"]),
        )

    def GetAllOrbita2d(self, request, context):
        # TODO
        return super().GetAllOrbita2d(request, context)
