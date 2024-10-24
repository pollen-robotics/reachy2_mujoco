import reachy2_sdk_api.orbita3d_pb2_grpc as orbita3d_pb2_grpc
from components import Component
from reachy2_sdk_api.orbita3d_pb2 import Orbita3d
from reachy2_sdk_api.component_pb2 import ComponentId
from utils import axis_from_str


class Orbita3dServicer(orbita3d_pb2_grpc.Orbita3dServiceServicer):
    def __init__(self, bridge_node):
        self.bridge_node = bridge_node

    @classmethod
    def get_info(cls, orbita3d: Component) -> Orbita3d:
        return Orbita3d(
            id=ComponentId(
                id=orbita3d.id,
                name=orbita3d.name,
            ),
        )

    def GetAllOrbita3d(self, request, context):
        # TODO
        return super().GetAllOrbita3d(request, context)
