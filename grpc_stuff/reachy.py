from enum import Enum
import reachy2_sdk_api.reachy_pb2_grpc as reachy_pb2_grpc
from reachy2_sdk_api.reachy_pb2 import Reachy, ReachyId, ReachyState, ReachyStatus
from google.protobuf.timestamp_pb2 import Timestamp
import time
from reachy2_sdk_api.part_pb2 import PartId


def endless_timer_get_stream_works(func, request, context, period):
    try:
        while True:
            yield func(request, context)
            time.sleep(period)
    except GeneratorExit:
        print("Client left stream!! Version with time.sleep()")
        raise


class ReachyServicer(reachy_pb2_grpc.ReachyServiceServicer):
    def __init__(
        self,
        bridge_node,
        arm_servicer,
        hand_servicer,
        head_servicer,
        mobile_base_servicer,
    ):
        self.bridge_node = bridge_node
        self.arm_servicer = arm_servicer
        self.hand_servicer = hand_servicer
        self.head_servicer = head_servicer
        self.mobile_base_servicer = mobile_base_servicer

        self.reachy_id = ReachyId(id=1, name="reachy")

    def GetReachy(self, request, context) -> Reachy:
        params = {
            "id": self.reachy_id,
        }

        for p in self.bridge_node.parts:
            if p.type == "arm":
                params[p.name] = self.arm_servicer.get_arm(p, context)
            elif p.type == "head":
                params[p.name] = self.head_servicer.get_head(p, context)
            elif p.type == "hand":
                params[p.name] = self.hand_servicer.get_hand(p, context)

        if self.mobile_base_servicer.get_mobile_base(context) is not None:
            params["mobile_base"] = self.mobile_base_servicer.get_mobile_base(context)

        print("coucou")

        return Reachy(**params)

    def GetReachyState(self, request, context) -> ReachyState:
        params = {
            "timestamp": Timestamp(),
            "id": ReachyId(id=1, name="reachy"),
        }

        for p in self.bridge_node.parts:
            if p.type == "arm":
                try:
                    params[f"{p.name}_state"] = self.arm_servicer.GetState(
                        PartId(id=p.id), context
                    )
                except Exception as e:
                    print(e)
                    exit()
            elif p.type == "head":
                params[f"{p.name}_state"] = self.head_servicer.GetState(
                    PartId(id=p.id), context
                )
            elif p.type == "hand":
                params[f"{p.name}_state"] = self.hand_servicer.GetState(
                    PartId(id=p.id), context
                )

        params["mobile_base_state"] = self.mobile_base_servicer.GetState(
            PartId(id=100), context
        )
        print("===")
        print("===")
        print("===")
        print("===")
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
