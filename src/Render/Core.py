## =========================================================================== ##
# MIT License
# Copyright (c) 2026 JIC Smart Factory
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
# The render flags of the studio look (F7) that are not model fields: fog (the floor fades into the horizon colour;
# MuJoCo's default is off) and haze (MuJoCo's default is on, kept on explicitly).
#   Unit [Dict<int, bool>] (mjtRndFlag -> on)
CONST_LOOK_FLAGS = {mujoco.mjtRndFlag.mjRND_FOG: True, mujoco.mjtRndFlag.mjRND_HAZE: True}


class Render_Error_Cls(ValueError):
    """
    Description:
        An image cannot be made: an unknown camera or a size beyond the model's offscreen buffer. The
        message names the value and the limit.
    """


def _Set_Look_Flags(flags: np.ndarray) -> None:
    """
    Description:
        Switch on the studio look's render flags (fog, haze) in a scene's flags.

    Args:
        (1) flags [Vector<uint8>]: An mjvScene's flags, changed in place.
    """

    for flag, on in CONST_LOOK_FLAGS.items():
        flags[flag] = on


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
        _Set_Look_Flags(renderer.scene.flags)
        return renderer.render().copy()
    finally:
        renderer.close()


def Render_Free_Image(model: mujoco.MjModel, data: mujoco.MjData, lookat: tp.Sequence[float], distance: float, azimuth: float,
                      elevation: float, width: int = CONST_WIDTH, height: int = CONST_HEIGHT) -> np.ndarray:
    """
    Description:
        An offscreen image of the data's current state from a free camera (a point looked at, a distance and two
        angles, as the viewer's mouse camera).

    Args:
        (1 - 2) model, data [mujoco.MjModel, mujoco.MjData]: The model and its state (after mj_forward).
        (3) lookat [Vector<float> 1x3]: The point looked at (m).
        (4) distance [float]: The camera's distance from it (m).
        (5 - 6) azimuth, elevation [float]: The camera's angles (deg; elevation negative = from above).
        (7 - 8) width, height [int]: The size (px).

    Returns:
        (1) parameter [Matrix<uint8> HxWx3]: The RGB image.
    """

    if width > model.vis.global_.offwidth or height > model.vis.global_.offheight:
        raise Render_Error_Cls('An image of %dx%d px exceeds the model\'s offscreen buffer %dx%d px.'
                               % (width, height, model.vis.global_.offwidth, model.vis.global_.offheight))
    camera = mujoco.MjvCamera()
    camera.lookat[:] = lookat
    camera.distance, camera.azimuth, camera.elevation = distance, azimuth, elevation
    renderer = mujoco.Renderer(model, height, width)
    try:
        renderer.update_scene(data, camera=camera)
        _Set_Look_Flags(renderer.scene.flags)
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
        runs under python on macOS; launch_passive would need mjpython). This viewer takes no render flags from
        code: the studio look's fog is off until switched on in the left panel, Rendering > Fog (Haze is on).

    Args:
        (1) model [mujoco.MjModel]: The model.
        (2) data [mujoco.MjData]: Its state (default: a new one).
    """

    import mujoco.viewer
    mujoco.viewer.launch(model, mujoco.MjData(model) if data is None else data)


def Launch_Passive_Viewer(model: mujoco.MjModel, data: mujoco.MjData, key_callback: tp.Optional[tp.Callable[[int], None]] = None,
                          command: str = '') -> tp.Any:
    """
    Description:
        Open MuJoCo's passive viewer on the data without stepping it (a posed cell stays as posed), with the studio
        look's fog and haze on. On macOS it runs only under mjpython: then a Render_Error_Cls names the command to use.

    Args:
        (1 - 2) model, data [mujoco.MjModel, mujoco.MjData]: The model and its posed state.
        (3) key_callback [function]: Called with each key code (GLFW), or None.
        (4) command [string]: The script and its arguments, for the mjpython hint.

    Returns:
        (1) parameter [mujoco.viewer.Handle]: The viewer (sync, lock, is_running, close).
    """

    import mujoco.viewer
    try:
        viewer = mujoco.viewer.launch_passive(model, data, key_callback=key_callback, show_left_ui=False, show_right_ui=False)
    except RuntimeError as error:
        raise Render_Error_Cls('The viewer cannot start (%s). On macOS run it under mjpython: .venv/bin/mjpython %s'
                               % (error, command)) from error
    # The studio look's fog and haze: the viewer copies its user scene's flags into its own scene at the next sync.
    with viewer.lock():
        _Set_Look_Flags(viewer.user_scn.flags)
    viewer.sync()
    return viewer

