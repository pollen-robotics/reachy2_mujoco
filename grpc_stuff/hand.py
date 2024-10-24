import grpc

import reachy2_sdk_api.hand_pb2_grpc as hand_pb2_grpc
from reachy2_sdk_api.part_pb2 import PartId
from parts import Part
from reachy2_sdk_api.hand_pb2 import Hand


class HandServicer(hand_pb2_grpc.HandServiceServicer):
    def __init__(self, bridge_node):
        self.bridge_node = bridge_node

    def get_hand(self, hand: Part, context: grpc.ServicerContext) -> Hand:
        return Hand(
            part_id=PartId(name=hand.name, id=hand.id),
        )

    def GetAllHands(self, request, context):
        # TODO
        return super().GetAllHands(request, context)
