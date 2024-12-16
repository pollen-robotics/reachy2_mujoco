import rpyc
from reachy2_mujoco import ReachyMujoco


class ReachySDK(ReachyMujoco):
    def __new__(cls, ip) -> ReachyMujoco:
        conn = rpyc.connect(ip, port=18861, config={"allow_pickle": True, "allow_all_attrs": True})
        reachy: ReachyMujoco = conn.root.reachy

        return reachy
