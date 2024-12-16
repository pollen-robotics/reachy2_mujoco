import grpc

import reachy2_sdk_api.head_pb2_grpc as head_pb2_grpc
from reachy2_sdk_api.head_pb2 import Head, HeadDescription

from reachy2_sdk_api.part_pb2 import PartId
from parts import Part
from orbita3d import Orbita3dServicer


class HeadServicer(head_pb2_grpc.HeadServiceServicer):
    def __init__(self, bridge_node):
        self.bridge_node = bridge_node

    def GetAllHeads(self, request, context):
        # TODO
        return super().GetAllHeads(request, context)

    def get_head(self, head: Part, context: grpc.ServicerContext) -> Head:
        return Head(
            part_id=PartId(name=head.name, id=head.id),
            description=HeadDescription(
                neck=Orbita3dServicer.get_info(self.bridge_node.components.get_by_name(head.components[0].name)),
                # l_antenna=DynamixelMotor.get_info(
                #     self.bridge_node.components.get_by_name(head.components[1].name)
                # ),
                # r_antenna=DynamixelMotor.get_info(
                #     self.bridge_node.components.get_by_name(head.components[2].name)
                # ),
            ),
        )
