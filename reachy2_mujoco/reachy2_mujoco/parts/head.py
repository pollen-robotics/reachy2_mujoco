from ..parts import Neck


class Head:
    def __init__(self, model, data):
        self._model = model
        self._data = data

        self.neck = Neck(self._model, self._data)

    def _update(self):
        self.neck._update()

    def _reset(self):
        self.neck._reset()


    def turn_on(self):
        self.neck.turn_on()
        return True


    def turn_off(self):
        self.neck.turn_off()
        return True

    def turn_off_smoothly(self):
        #TODO
        pass

    def is_on(self):
        self.neck.is_on()

    def is_off(self):
        self.neck.is_off()
