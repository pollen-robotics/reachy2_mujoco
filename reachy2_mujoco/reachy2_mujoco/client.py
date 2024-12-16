import rpyc
from reachy2_mujoco import ReachyMujoco


class ReachySDK(object):
    def __new__(cls, ip) -> ReachyMujoco:
        conn = rpyc.connect(ip, port=18861)
        reachy: ReachyMujoco = conn.root.reachy

        return reachy
