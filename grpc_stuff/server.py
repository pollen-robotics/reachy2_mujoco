import grpc
from concurrent import futures
import time
import reachy2_sdk_api.reachy_pb2_grpc as reachy_pb2_grpc
import reachy2_sdk_api.arm_pb2_grpc as arm_pb2_grpc
from reachy2_sdk_api.reachy_pb2 import Reachy, ReachyId, ReachyState, ReachyStatus
from google.protobuf.timestamp_pb2 import Timestamp

from reachy2_sdk_api.arm_pb2 import Arm, ArmDescription
from reachy2_sdk.parts.part import Part
from reachy2_sdk_api.part_pb2 import PartId

import reachy2_sdk_api.orbita2d_pb2_grpc as orbita2d_pb2_grpc
import reachy2_sdk_api.orbita3d_pb2_grpc as orbita3d_pb2_grpc

# from reachy2_sdk.orbita.orbita2d import (
#     ComponentId,
#     Orbita2dCommand,
#     Orbita2dsCommand,
#     Orbita2dServicer,
#     Orbita2dStateRequest,
# )
# from reachy2_sdk.orbita.orbita3d import (
#     Orbita3dCommand,
#     Orbita3dsCommand,
#     Orbita3dServicer,
#     Orbita3dStateRequest,
# )


def endless_timer_get_stream_works(func, request, context, period):
    try:
        while True:
            yield func(request, context)
            time.sleep(period)
    except GeneratorExit:
        print("Client left stream!! Version with time.sleep()")
        raise


class Orbita2dServicer(orbita2d_pb2_grpc.Orbita2dServiceServicer):
    def GetAllOrbita2d(self, request, context):
        # TODO
        return super().GetAllOrbita2d(request, context)


class Orbita3dServicer(orbita3d_pb2_grpc.Orbita3dServiceServicer):
    def GetAllOrbita3d(self, request, context):
        # TODO
        return super().GetAllOrbita3d(request, context)


class ArmServicer(arm_pb2_grpc.ArmServiceServicer):
    def GetAllArms(self, request, context):
        # TODO
        return super().GetAllArms(request, context)

    # TODO
    # def get_arm(self, arm: Part, context: grpc.ServicerContext) -> Arm:
    #     return Arm(
    #         part_id=PartId(name=arm.name, id=arm.id),
    #         description=ArmDescription(
    #             shoulder=Orbita2dServicer.get_info(
    #                 self.bridge_node.components.get_by_name(arm.components[0].name)
    #             ),
    #             elbow=Orbita2dServicer.get_info(
    #                 self.bridge_node.components.get_by_name(arm.components[1].name)
    #             ),
    #             wrist=Orbita3dServicer.get_info(
    #                 self.bridge_node.components.get_by_name(arm.components[2].name)
    #             ),
    #         ),
    #     )


class ReachyServicer(reachy_pb2_grpc.ReachyServiceServicer):
    def GetReachy(self, request, context) -> Reachy:
        params = {
            "id": ReachyId(id=1, name="reachy"),
            # "l_arm": Arm(),
            # "r_arm": None,
            # "head": None,
        }
        print("coucou")

        return Reachy(**params)

    def GetReachyState(self, request, context) -> ReachyState:
        params = {
            "timestamp": Timestamp(),
            "id": ReachyId(id=1, name="reachy"),
        }
        return ReachyState(**params)

    def Audit(self, request, context) -> ReachyStatus:
        params = {
            "timestamp": Timestamp(),
            "id": ReachyId(id=1, name="reachy"),
        }
        return ReachyStatus(**params)

    def StreamReachyState(self, request, context):
        return endless_timer_get_stream_works(
            self.GetReachyState,
            request.id,
            context,
            1 / request.publish_frequency,
        )


def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    reachy_pb2_grpc.add_ReachyServiceServicer_to_server(ReachyServicer(), server)
    server.add_insecure_port("[::]:50051")
    server.start()
    server.wait_for_termination()


if __name__ == "__main__":
    serve()
