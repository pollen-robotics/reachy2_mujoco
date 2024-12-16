import rpyc
from rpyc.utils.server import ThreadedServer
from reachy2_mujoco import ReachyMujoco


class ReachyMujocoService(rpyc.Service):
    exposed_reachy = ReachyMujoco()

    def on_connect(self, conn):
        pass

    def on_disconnect(self, conn):
        pass


def main():
    t = ThreadedServer(
        ReachyMujocoService,
        port=18861,
        protocol_config={
            "allow_all_attrs": True,
            "allow_setattr": True,
            # "allow_delattr": True,
        },
    )
    t.start()


if __name__ == "__main__":
    main()
