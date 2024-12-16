import grpc
import reachy2_sdk_api.arm_pb2_grpc as arm_pb2_grpc
from reachy2_sdk_api.arm_pb2 import Arm, ArmDescription
from reachy2_sdk_api.part_pb2 import PartId
from parts import Part
from orbita2d import Orbita2dServicer
from orbita3d import Orbita3dServicer
from utils import get_current_timestamp
from reachy2_sdk_api.arm_pb2 import ArmState

# from orbita2d import Orbita2dStateRequest
# from orbita3d import Orbita3dStateRequest
from reachy2_sdk_api.component_pb2 import ComponentId
from reachy2_sdk_api.orbita2d_pb2 import Orbita2dStateRequest
from reachy2_sdk_api.orbita3d_pb2 import Orbita3dStateRequest


class ArmServicer(arm_pb2_grpc.ArmServiceServicer):
    def __init__(self, bridge_node, orbita2d_servicer, orbita3d_servicer):
        self.bridge_node = bridge_node
        self.orbita2d_servicer = orbita2d_servicer
        self.orbita3d_servicer = orbita3d_servicer

        self.arms = self.bridge_node.parts.get_by_type("arm")

    # TODO
    def get_arm(self, arm: Part, context: grpc.ServicerContext) -> Arm:
        return Arm(
            part_id=PartId(name=arm.name, id=arm.id),
            description=ArmDescription(
                shoulder=Orbita2dServicer.get_info(self.bridge_node.components.get_by_name(arm.components[0].name)),
                elbow=Orbita2dServicer.get_info(self.bridge_node.components.get_by_name(arm.components[1].name)),
                wrist=Orbita3dServicer.get_info(self.bridge_node.components.get_by_name(arm.components[2].name)),
            ),
        )

    def get_arm_part_by_part_id(self, part_id: PartId, context: grpc.ServicerContext) -> Part:
        part = self.bridge_node.parts.get_by_part_id(part_id)

        if part is None:
            context.abort(grpc.StatusCode.NOT_FOUND, f"Part not found (id={part_id}).")

        if part.type != "arm":
            context.abort(
                grpc.StatusCode.INVALID_ARGUMENT,
                f"Part '{part_id}' is not an arm.",
            )
        return part

    def GetAllArms(self, request, context):
        # TODO
        return super().GetAllArms(request, context)

    def GetState(self, request, context):
        arm = self.get_arm_part_by_part_id(request, context)
        arm_state = ArmState(
            timestamp=get_current_timestamp(self.bridge_node),
            id=request,
            activated=True,
            shoulder_state=self.orbita2d_servicer.GetState(
                Orbita2dStateRequest(
                    fields=self.orbita2d_servicer.default_fields,
                    id=ComponentId(id=arm.components[0].id),
                ),
                context,
            ),
            elbow_state=self.orbita2d_servicer.GetState(
                Orbita2dStateRequest(
                    fields=self.orbita2d_servicer.default_fields,
                    id=ComponentId(id=arm.components[1].id),
                ),
                context,
            ),
            wrist_state=self.orbita3d_servicer.GetState(
                Orbita3dStateRequest(
                    fields=self.orbita3d_servicer.default_fields,
                    id=ComponentId(id=arm.components[2].id),
                ),
                context,
            ),
            reachability=self.get_reachability_state(arm.name),
        )
        return arm_state
