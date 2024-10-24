import reachy2_sdk_api.hand_pb2_grpc as hand_pb2_grpc


class HandServicer(hand_pb2_grpc.HandServiceServicer):
    def __init__(self, bridge_node):
        self.bridge_node = bridge_node

    def GetAllHands(self, request, context):
        # TODO
        return super().GetAllHands(request, context)
