import mujoco
import numpy as np


# TODO implement reachy.cameras.teleop.get_frame(left/right)
class Camera:
    def __init__(self, model, data, cam_name, width, height, fps=30):
        self._model = model
        self._data = data
        self._cam_name = cam_name
        self._width = width
        self._height = height
        self._fps = fps

        self._camera_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_CAMERA, self._cam_name)
        self._offscreen = mujoco.Renderer(self._model)
        self._rgb_array = np.zeros((self._height, self._width, 3), dtype=np.uint8)

    def _update(self):
        self._offscreen.update_scene(self._data, self._camera_id)
        self._offscreen.render(out=self._rgb_array)

    def get_image(self):
        return self._rgb_array
