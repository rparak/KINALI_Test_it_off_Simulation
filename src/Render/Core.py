## =========================================================================== ##
# MIT License
# Copyright (c) 2026 Roman Parak
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
## =========================================================================== ##
# Author   : Roman Parak
# File Name: ../Render/Core.py
## =========================================================================== ##

# System (Default)
import os
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco

"""
Description:
    Initialization of constants.

    Pictures of a model (SPEC §7.5): an offscreen image from a named camera, a PNG file of it, the
    interactive viewer. It knows nothing of the cell: it takes a model and its data. matplotlib (the PNG
    writer) and mujoco.viewer (a GUI) are imported only where they are used.
"""
# Default offscreen image size; within the scene's offscreen buffer (offwidth 1280, offheight 960).
#   Unit [int] (px)
CONST_WIDTH, CONST_HEIGHT = 960, 720


class Render_Error_Cls(ValueError):
    """
    Description:
        An image cannot be made: an unknown camera or a size beyond the model's offscreen buffer. The
        message names the value and the limit.
    """


def Render_Image(model: mujoco.MjModel, data: mujoco.MjData, camera: str, width: int = CONST_WIDTH, height: int = CONST_HEIGHT,
                 segmentation: bool = False) -> np.ndarray:
    """
    Description:
        An offscreen image of the data's current state from a named camera of the model.

    Args:
        (1 - 2) model, data [mujoco.MjModel, mujoco.MjData]: The model and its state (after mj_forward).
        (3) camera [string]: The camera name (e.g. cam_pcb, cam_tool).
        (4 - 5) width, height [int]: The size (px).
        (6) segmentation [bool]: Return the segmentation image (object id, object type per pixel) instead.

    Returns:
        (1) parameter [Matrix<uint8> HxWx3]: The RGB image (or, with segmentation, int32 ids HxWx2).
    """

    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera) < 0:
        names = [model.camera(i).name for i in range(model.ncam)]
        raise Render_Error_Cls("The model has no camera '%s'; its cameras are %s." % (camera, ', '.join(names)))
    if width > model.vis.global_.offwidth or height > model.vis.global_.offheight:
        raise Render_Error_Cls('An image of %dx%d px exceeds the model\'s offscreen buffer %dx%d px.'
                               % (width, height, model.vis.global_.offwidth, model.vis.global_.offheight))
    renderer = mujoco.Renderer(model, height, width)
    try:
        if segmentation:
            renderer.enable_segmentation_rendering()
        renderer.update_scene(data, camera=camera)
        return renderer.render().copy()
    finally:
        renderer.close()


def Save_Image(image: np.ndarray, path: str) -> str:
    """
    Description:
        Write an RGB image as a PNG file (matplotlib), creating its folder.

    Args:
        (1) image [Matrix<uint8> HxWx3]: The image.
        (2) path [string]: The file.

    Returns:
        (1) parameter [string]: The absolute path written.
    """

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.image as mpimg
    path = os.path.abspath(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    mpimg.imsave(path, image)
    return path


def Launch_Viewer(model: mujoco.MjModel, data: tp.Optional[mujoco.MjData] = None) -> None:
    """
    Description:
        Open MuJoCo's interactive viewer on the model and block until it is closed (mujoco.viewer.launch:
        runs under python on macOS; launch_passive would need mjpython).

    Args:
        (1) model [mujoco.MjModel]: The model.
        (2) data [mujoco.MjData]: Its state (default: a new one).
    """

    import mujoco.viewer
    mujoco.viewer.launch(model, mujoco.MjData(model) if data is None else data)
