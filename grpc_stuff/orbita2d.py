import reachy2_sdk_api.orbita2d_pb2_grpc as orbita2d_pb2_grpc


class Orbita2dServicer(orbita2d_pb2_grpc.Orbita2dServiceServicer):
    def __init__(self, bridge_node):
        self.bridge_node = bridge_node

    def GetAllOrbita2d(self, request, context):
        # TODO
        return super().GetAllOrbita2d(request, context)
