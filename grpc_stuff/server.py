import grpc
from concurrent import futures
import reachy2_sdk_api.reachy_pb2_grpc as reachy_pb2_grpc

from mujoco_bridge_node import MujocoBridgeNode


from arm import ArmServicer
from reachy import ReachyServicer
from mobile_base import MobileBaseMobilityServicer
from orbita2d import Orbita2dServicer
from orbita3d import Orbita3dServicer
from head import HeadServicer
from hand import HandServicer

joint_server_port = 50051


def serve():
    mujoco_bridge_node = MujocoBridgeNode("../reachy.yaml")
    orbita2d_servicer = Orbita2dServicer(mujoco_bridge_node)
    orbita3d_servicer = Orbita3dServicer(mujoco_bridge_node)
    arm_servicer = ArmServicer(mujoco_bridge_node, orbita2d_servicer, orbita3d_servicer)
    hand_servicer = HandServicer(mujoco_bridge_node)
    head_servicer = HeadServicer(mujoco_bridge_node)
    mobile_base_servicer = MobileBaseMobilityServicer(mujoco_bridge_node)

    reachy_servicer = ReachyServicer(
        mujoco_bridge_node,
        arm_servicer,
        hand_servicer,
        head_servicer,
        mobile_base_servicer,
    )

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    reachy_pb2_grpc.add_ReachyServiceServicer_to_server(reachy_servicer, server)
    server.add_insecure_port(f"[::]:{joint_server_port}")
    server.start()
    print(f"waiting on {joint_server_port}...")
    server.wait_for_termination()


if __name__ == "__main__":
    serve()
