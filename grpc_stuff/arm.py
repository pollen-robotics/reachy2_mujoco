import grpc
import reachy2_sdk_api.arm_pb2_grpc as arm_pb2_grpc
from reachy2_sdk_api.arm_pb2 import Arm, ArmDescription
from reachy2_sdk_api.part_pb2 import PartId
from parts import Part
from orbita2d import Orbita2dServicer
from orbita3d import Orbita3dServicer


class ArmServicer(arm_pb2_grpc.ArmServiceServicer):
    def __init__(self, bridge_node, orbita2d_servicer, orbita3d_servicer):
        self.bridge_node = bridge_node
        self.orbita2d_servicer = orbita2d_servicer
        self.orbita3d_servicer = orbita3d_servicer

        self.arms = self.bridge_node.parts.get_by_type("arm")

    def GetAllArms(self, request, context):
        # TODO
        return super().GetAllArms(request, context)

    # TODO
    def get_arm(self, arm: Part, context: grpc.ServicerContext) -> Arm:
        return Arm(
            part_id=PartId(name=arm.name, id=arm.id),
            description=ArmDescription(
                shoulder=Orbita2dServicer.get_info(
                    self.bridge_node.components.get_by_name(arm.components[0].name)
                ),
                elbow=Orbita2dServicer.get_info(
                    self.bridge_node.components.get_by_name(arm.components[1].name)
                ),
                wrist=Orbita3dServicer.get_info(
                    self.bridge_node.components.get_by_name(arm.components[2].name)
                ),
            ),
        )
