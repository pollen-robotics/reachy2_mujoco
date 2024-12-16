import grpc
import threading
import asyncio

from reachy_sdk_server.reachy_sdk_server.grpc_server.arm import ArmServicer
from reachy_sdk_server.reachy_sdk_server.grpc_server.goto import GoToServicer
from reachy_sdk_server.reachy_sdk_server.grpc_server.hand import HandServicer
from reachy_sdk_server.reachy_sdk_server.grpc_server.head import HeadServicer
from reachy_sdk_server.reachy_sdk_server.grpc_server.mobile_base import (
    MobileBaseServicer,
)
from reachy_sdk_server.reachy_sdk_server.grpc_server.orbita2d import Orbita2dServicer
from reachy_sdk_server.reachy_sdk_server.grpc_server.orbita3d import Orbita3dServicer
from reachy_sdk_server.reachy_sdk_server.grpc_server.reachy import ReachyServicer


class ReachyGRPCJointSDKServicer:
    def __init__(self, reachy_config_path: str = None, port=None) -> None:
        rclpy.init()

        self.asyncio_loop = asyncio.new_event_loop()

        self.bridge_node = AbstractBridgeNode(
            reachy_config_path=reachy_config_path,
            asyncio_loop=self.asyncio_loop,
            port=port,
        )

        self.asyncio_thread = threading.Thread(target=self.spin_asyncio)
        self.asyncio_thread.start()

        # sanity check loop running at 1Hz
        self.asyncio_loop_sanity = asyncio.new_event_loop()
        self.asyncio_thread_sanity = threading.Thread(target=self.spin_asyncio_sanity)
        self.asyncio_thread_sanity.start()

        self.logger = self.bridge_node.get_logger()

        orbita2d_servicer = Orbita2dServicer(self.bridge_node, self.logger)
        orbita3d_servicer = Orbita3dServicer(self.bridge_node, self.logger)
        arm_servicer = ArmServicer(
            self.bridge_node,
            self.logger,
            orbita2d_servicer,
            orbita3d_servicer,
        )
        hand_servicer = HandServicer(self.bridge_node, self.logger)
        head_servicer = HeadServicer(self.bridge_node, self.logger, orbita3d_servicer)
        goto_servicer = GoToServicer(self.bridge_node, self.logger)
        mobile_base_servicer = MobileBaseServicer(self.bridge_node, self.logger, reachy_config_path)
        reachy_servicer = ReachyServicer(
            self.bridge_node,
            self.logger,
            arm_servicer,
            hand_servicer,
            head_servicer,
            mobile_base_servicer,
        )

        self.services = [
            arm_servicer,
            goto_servicer,
            hand_servicer,
            head_servicer,
            mobile_base_servicer,
            orbita2d_servicer,
            orbita3d_servicer,
            reachy_servicer,
        ]

        self.logger.info("Reachy GRPC Joint SDK Servicer initialized.")

    def register_all_services(self, server: grpc.Server) -> None:
        for serv in self.services:
            serv.register_to_server(server)

    def spin_asyncio(self) -> None:
        asyncio.set_event_loop(self.asyncio_loop)
        self.asyncio_loop.run_until_complete(self.spinning(self.bridge_node))

    def spin_asyncio_sanity(self) -> None:
        asyncio.set_event_loop(self.asyncio_loop_sanity)
        self.asyncio_loop_sanity.run_until_complete(self.spinning_sanity(self.bridge_node))

    async def spinning_sanity(self, node):
        with node.sum_spin_sanity.time():
            await asyncio.sleep(1)

    async def spinning(self, node):
        while rclpy.ok():
            with node.sum_spin.time():
                rclpy.spin_once(node, timeout_sec=0.01)
            await asyncio.sleep(0.001)


def main_singleprocess(_=1):
    import argparse
    from concurrent import futures

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--max-workers", type=int, default=10)
    parser.add_argument("--ros-args", action="store_true")
    parser.add_argument("reachy_config", type=str, default="../reachy.yaml")
    args = parser.parse_args()

    port = f"{args.port}{_}"
    print(f"Starting grpc server at {port}")

    servicer = ReachyGRPCJointSDKServicer(reachy_config_path=args.reachy_config, port=port)
    server = grpc.server(
        futures.ThreadPoolExecutor(max_workers=args.max_workers),
        # interceptors=[Interceptor()],
    )

    servicer.register_all_services(server)

    server.add_insecure_port(f"[::]:{port}")
    print("Ready to start")
    server.start()

    # servicer.logger.info(f"Server started on port {port}.")
    print(f"Server started on port {port}.")
    server.wait_for_termination()


if __name__ == "__main__":
    main_singleprocess()
