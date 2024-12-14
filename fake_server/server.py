import threading
import time
import numpy as np  
import rpyc
from rpyc.utils.server import ThreadedServer


class FooRobot():
    def __init__(self):
        self.shoulder_roll = 0
        self.shoulder_pitch = 0
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def exposed_get_shoulder_roll(self):
        return self.shoulder_roll

    def exposed_get_shoulder_pitch(self):
        return self.shoulder_pitch

    def exposed_set_shoulder_pitch(self, value):
        self.shoulder_pitch = value

    def run(self):
        while True:
            self.shoulder_roll = np.random.rand()
            print("Shoulder roll: ", self.shoulder_roll)
            print("Shoulder pitch: ", self.shoulder_pitch)
            print("==")
            time.sleep(1)


class MyService(rpyc.Service):

    exposed_robot = FooRobot()

    def on_connect(self, conn):
        pass

    def on_disconnect(self, conn):
        pass


if __name__ == "__main__":

    t = ThreadedServer(MyService, port=18861)
    t.start()