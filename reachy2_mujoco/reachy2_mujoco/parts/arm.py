from reachy2_mujoco.parts.joints import Shoulder, Elbow, Wrist, Gripper


class Arm:
    def __init__(self, model, data, prefix="l_"):
        self._model = model
        self._data = data

        self.shoulder = Shoulder(self._model, self._data, prefix=prefix)
        self.elbow = Elbow(self._model, self._data, prefix=prefix)
        self.wrist = Wrist(self._model, self._data, prefix=prefix)

        self.gripper = Gripper(self._model, self._data, prefix=prefix)

    def _update(self):
        self.shoulder._update()
        self.elbow._update()
        self.wrist._update()
        self.gripper._update()

    # TODO implement
    def goto(self):
        print("Not implemented")
        pass
