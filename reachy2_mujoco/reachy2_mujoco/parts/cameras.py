import mujoco
import numpy as np
import cv2
import time
from enum import IntEnum


class CameraView(IntEnum):
    LEFT = 0
    RIGHT = 1
    DEPTH = 2


# TODO implement reachy.cameras.teleop.get_frame(left/right)
class Camera:
    def __init__(self, model, data, cam_name, width, height, offscreen: mujoco.Renderer, fps=5):
        self._model = model
        self._data = data
        self._cam_name = cam_name
        self._width = width
        self._height = height
        self._offscreen = offscreen
        self._fps = fps

        self._last_update = time.time()

        self._left_camera_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_CAMERA, "left_" + self._cam_name)
        self._right_camera_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_CAMERA, "right_" + self._cam_name)
        self._left_im = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        self._right_im = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        self._depth_im = np.zeros((self._height, self._width), dtype=np.float32)

    def _update(self):
        # This has a huge impact on the performance. Keep the fps low
        # return
        if time.time() - self._last_update < 1 / self._fps:
            return
        self._offscreen.update_scene(self._data, self._left_camera_id)
        self._left_im = self._offscreen.render()
        # return
        self._offscreen.enable_depth_rendering()
        # self._offscreen.update_scene(self._data, self._left_camera_id)
        self._depth_im = self._offscreen.render()
        self._offscreen.disable_depth_rendering()
        # return
        # self._depth_im -= self._depth_im.min()
        # self._depth_im /= 2 * self._depth_im[self._depth_im <= 1].mean()
        # self._depth_im = 255 * np.clip(self._depth_im, 0, 1)

        self._offscreen.update_scene(self._data, self._right_camera_id)
        self._offscreen.render(out=self._right_im)

    def get_frame(self, view: CameraView = CameraView.LEFT):
        if view.value == CameraView.LEFT.value:
            return np.array(cv2.cvtColor(self._left_im, cv2.COLOR_RGB2BGR))
        elif view.value == CameraView.RIGHT.value:
            return cv2.cvtColor(self._right_im, cv2.COLOR_RGB2BGR)
        elif view.value == CameraView.DEPTH.value:
            return self._depth_im
        else:
            raise ValueError("Unknown view")


class Cameras:
    def __init__(self, model, data, width, height):
        self._model = model
        self._data = data
        self._width = width
        self._height = height
        self._offscreen = mujoco.Renderer(self._model, height=self._height, width=self._width)
        # self._offscreen.enable_depth_rendering()

        self.teleop = Camera(self._model, self._data, "teleop_cam", self._width, self._height, self._offscreen)

    def _update(self):
        self.teleop._update()
