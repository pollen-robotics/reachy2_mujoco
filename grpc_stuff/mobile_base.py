import reachy2_sdk_api.mobile_base_mobility_pb2_grpc as mobile_base_mobility_pb2_grpc


class MobileBaseMobilityServicer(
    mobile_base_mobility_pb2_grpc.MobileBaseMobilityServiceServicer
):
    def __init__(self, bridge_node):
        self.bridge_node = bridge_node

    def GetAllMobileBaseMobility(self, request, context):
        # TODO
        return super().GetAllMobileBaseMobility(request, context)
