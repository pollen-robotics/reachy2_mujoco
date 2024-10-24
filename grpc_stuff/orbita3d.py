import reachy2_sdk_api.orbita3d_pb2_grpc as orbita3d_pb2_grpc


class Orbita3dServicer(orbita3d_pb2_grpc.Orbita3dServiceServicer):
    def __init__(self, bridge_node):
        self.bridge_node = bridge_node

    def GetAllOrbita3d(self, request, context):
        # TODO
        return super().GetAllOrbita3d(request, context)
