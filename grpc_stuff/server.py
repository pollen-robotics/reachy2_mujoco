import grpc
from concurrent import futures
import time
import reachy2_sdk_api.reachy_pb2_grpc as reachy_pb2_grpc
from reachy2_sdk_api.reachy_pb2 import Reachy, ReachyId, ReachyState, ReachyStatus
from google.protobuf.timestamp_pb2 import Timestamp

from reachy2_sdk_api.arm_pb2 import Arm


def endless_timer_get_stream_works(func, request, context, period):
    try:
        while True:
            yield func(request, context)
            time.sleep(period)
    except GeneratorExit:
        print("Client left stream!! Version with time.sleep()")
        raise


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
