import grpc

import reachy2_sdk_api.mobile_base_mobility_pb2_grpc as mobile_base_mobility_pb2_grpc
from reachy2_sdk_api.mobile_base_utility_pb2 import MobileBase
from reachy2_sdk_api.part_pb2 import PartId, PartInfo


class MobileBaseMobilityServicer(
    mobile_base_mobility_pb2_grpc.MobileBaseMobilityServiceServicer
):
    def __init__(self, bridge_node):
        self.bridge_node = bridge_node
        self.mobile_base_enabled = True
        self._part_id = PartId(id=100, name="mobile_base")

    def get_mobile_base(self, context: grpc.ServicerContext) -> MobileBase:
        """Get mobile base basic info."""
        if self.mobile_base_enabled:
            return MobileBase(
                part_id=self._part_id,
                info=PartInfo(
                    serial_number="MUJOCO", version_hard="MUJOCO", version_soft="MUJOCO"
                ),
            )
        else:
            return None

    def GetAllMobileBaseMobility(self, request, context):
        # TODO
        return super().GetAllMobileBaseMobility(request, context)
