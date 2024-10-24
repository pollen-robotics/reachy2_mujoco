import reachy2_sdk_api.head_pb2_grpc as head_pb2_grpc


class HeadServicer(head_pb2_grpc.HeadServiceServicer):
    def __init__(self, bridge_node):
        self.bridge_node = bridge_node

    def GetAllHeads(self, request, context):
        # TODO
        return super().GetAllHeads(request, context)
