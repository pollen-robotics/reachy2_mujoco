import rpyc
from rpyc.utils.server import ThreadedServer
from reachy_mujoco import ReachyMujoco


class ReachyMujocoService(rpyc.Service):

    exposed_reachy = ReachyMujoco()

    def on_connect(self, conn):
        pass

    def on_disconnect(self, conn):
        pass


if __name__ == "__main__":

    t = ThreadedServer(ReachyMujocoService, port=18861)
    t.start()
