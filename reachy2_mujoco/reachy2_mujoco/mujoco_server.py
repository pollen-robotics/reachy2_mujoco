import rpyc
from rpyc.utils.server import ThreadedServer

from reachy2_mujoco import ReachyMujoco
import argparse

def main():

    ENV=''

    parser = argparse.ArgumentParser()
    parser.add_argument("--env", help="path to the scene xml", type=str, default='')
    args = parser.parse_args()
    print(args.env)
    ENV=args.env


    class ReachyMujocoService(rpyc.Service):

        exposed_reachy = ReachyMujoco(ENV)

        def on_connect(self, conn):
            pass
        def on_disconnect(self, conn):
            pass



    t = ThreadedServer(
        ReachyMujocoService,
        port=18861,
        protocol_config={
            "allow_all_attrs": True,
            "allow_setattr": True,
            "allow_pickle": True
            # "allow_delattr": True,
        },
    )
    t.start()


if __name__ == "__main__":
    main()
