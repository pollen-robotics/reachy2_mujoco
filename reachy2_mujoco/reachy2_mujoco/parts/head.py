from reachy2_mujoco.parts import Neck


class Head:
    def __init__(self, model, data):
        self._model = model
        self._data = data

        self.neck = Neck(self._model, self._data)

    def _update(self):
        self.neck._update()
