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
# File Name: ../test/Render/test_core.py
## =========================================================================== ##

# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Custom Lib.: the module under test
#   ../Render/Core
from Render.Core import Render_Image, Save_Image, Render_Error_Cls


@pytest.fixture(scope='module')
def state(cell_model):
    data = mujoco.MjData(cell_model)
    mujoco.mj_forward(cell_model, data)
    return cell_model, data


def test_the_pcb_camera_shows_a_green_pcb(state):
    """
    Description:
        cam_pcb at the initial state sees the PCB visual (segmentation), and its pixels are green: the
        green channel dominates (measured 2026-10-24: mean RGB 17.6 / 121.5 / 55.1 over 2398 px).
    """

    model, data = state
    image = Render_Image(model, data, 'cam_pcb')
    segmentation = Render_Image(model, data, 'cam_pcb', segmentation=True)
    assert image.shape == (720, 960, 3) and image.dtype == np.uint8
    mask = (segmentation[..., 0] == model.geom('pcb_vis').id) & (segmentation[..., 1] == mujoco.mjtObj.mjOBJ_GEOM)
    assert mask.sum() > 1000
    r, g, b = image[mask].mean(0)
    assert g > 2 * r and g > 1.5 * b


def test_a_saved_image_reads_back(state, tmp_path):
    """
    Description:
        Save_Image writes a PNG of the image's size into a new folder.
    """

    import matplotlib.image as mpimg
    model, data = state
    path = Save_Image(Render_Image(model, data, 'cam_tool', 320, 240), str(tmp_path / 'new' / 'tool.png'))
    assert mpimg.imread(path).shape[:2] == (240, 320)


def test_an_unknown_camera_or_a_too_large_image_is_refused(state):
    """
    Description:
        An unknown camera is named with the model's cameras; a size beyond the offscreen buffer is named.
    """

    model, data = state
    with pytest.raises(Render_Error_Cls, match="no camera 'cam_none'.*cam_pcb, cam_tool"):
        Render_Image(model, data, 'cam_none')
    with pytest.raises(Render_Error_Cls, match='2000x720 px exceeds'):
        Render_Image(model, data, 'cam_pcb', 2000, 720)


def test_a_free_camera_image_and_its_size_limit(state):
    """
    Description:
        Render_Free_Image looks at a point from a distance and two angles; a size beyond the offscreen buffer is
        refused.
    """

    from Render.Core import Render_Free_Image
    model, data = state
    image = Render_Free_Image(model, data, [0.0, -0.3, 0.2], 1.5, 150, -25, 320, 240)
    assert image.shape == (240, 320, 3) and image.std() > 0
    with pytest.raises(Render_Error_Cls, match='exceeds'):
        Render_Free_Image(model, data, [0.0, 0.0, 0.0], 1.0, 0, 0, 4000, 240)


def test_the_floor_grid_lines_are_one_metre_apart_on_whole_metres(state):
    """
    Description:
        Looking straight down from 3 m at (2.5, 2.5) on the floor (fovy 45 deg: 289.7 px/m), the grid lines across a
        row are 1.00 m apart and at x = 2.5 +- 0.5, +- 1.5 m: one line per metre on the world axes (measured
        2026-10-24: 289.5-290 px).
    """

    from Render.Core import Render_Free_Image
    model, data = state
    image = Render_Free_Image(model, data, [2.5, 2.5, -0.873], 3.0, 90, -90).astype(float).mean(2)
    row = image[300]
    base = np.median(row)
    columns = np.where(row > base + 0.5 * (row.max() - base))[0]
    lines = [g.mean() for g in np.split(columns, np.where(np.diff(columns) > 1)[0] + 1)]
    pixels_per_metre = image.shape[0] / (2 * 3.0 * np.tan(np.radians(model.vis.global_.fovy / 2)))
    offsets = (np.array(lines) - (image.shape[1] - 1) / 2) / pixels_per_metre
    assert offsets == pytest.approx([-1.5, -0.5, 0.5, 1.5], abs=0.01)


def test_the_offscreen_floor_fades_into_the_horizon_colour(state):
    """
    Description:
        Fog is on in the offscreen renders: near the horizon (a low view, left of the cell) the far floor has the
        horizon colour (0.42 0.45 0.49 = 115.6 / 255 mean), not its own dark grey (measured 2026-10-24: 115.7 with
        fog, about 51 without).
    """

    from Render.Core import Render_Free_Image
    model, data = state
    image = Render_Free_Image(model, data, [0.2, -0.3, -0.6], 3.0, 120, -4).astype(float)
    assert image[290:315, 60:120].mean() == pytest.approx(np.mean([0.42, 0.45, 0.49]) * 255, abs=2)


def test_the_passive_viewer_switches_on_fog_and_haze(state, monkeypatch):
    """
    Description:
        Launch_Passive_Viewer sets fog and haze in the viewer's user scene flags (copied into the viewer's scene at
        the sync) and syncs once; the GUI itself is replaced by a stand-in (no window in the tests).
    """

    import contextlib
    import types
    import mujoco.viewer
    from Render.Core import Launch_Passive_Viewer
    model, data = state
    handle = types.SimpleNamespace(user_scn=mujoco.MjvScene(model, 10), lock=contextlib.nullcontext, syncs=[])
    handle.sync = lambda: handle.syncs.append(1)
    monkeypatch.setattr(mujoco.viewer, 'launch_passive', lambda *args, **kwargs: handle)
    assert Launch_Passive_Viewer(model, data) is handle
    flags = handle.user_scn.flags
    assert flags[mujoco.mjtRndFlag.mjRND_FOG] == 1 and flags[mujoco.mjtRndFlag.mjRND_HAZE] == 1 and handle.syncs == [1]
