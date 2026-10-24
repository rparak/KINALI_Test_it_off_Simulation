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

# System (Default)
import os
import sys
import subprocess
# Threading (Thread-based parallelism)
import threading
# Types (Dynamic type creation and names for built-in types)
import types
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Custom Lib.: the module under test
#   ../Render/Core
from Render.Core import (Render_Image, Save_Image, Render_Error_Cls, Playback_Cls, Run_Viewer_Loop, Watchdog_Cls, CONST_FRAME_PERIOD,
                         CONST_OVERLAY_PERIOD, CONST_WATCHDOG_EXIT)


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


def _Fake_Viewer(frames):
    """
    Description:
        A passive viewer as mujoco 3.16 behaves (measured 2026-10-24 under mjpython: set_texts under viewer.lock() never
        returns): lock() is a non-reentrant lock, and set_texts, which waits for the render thread that needs that lock,
        fails when the lock is held instead of blocking (so does sync). It runs for a number of frames and records the calls.
    """

    viewer = types.SimpleNamespace(Lock=threading.Lock(), Frames=frames, Calls=[])

    def Is_Running():
        viewer.Frames -= 1
        return viewer.Frames >= 0

    def Unlocked(name, *arguments):
        if not viewer.Lock.acquire(blocking=False):
            raise RuntimeError('%s under viewer.lock(): the render thread can never take the lock (deadlock)' % name)
        viewer.Lock.release()
        viewer.Calls.append((name,) + arguments)

    viewer.is_running, viewer.lock = Is_Running, lambda: viewer.Lock
    viewer.set_texts = lambda texts: Unlocked('set_texts', texts[2], texts[3])
    viewer.sync = lambda: Unlocked('sync')
    return viewer


class _Clock_Cls(object):
    """
    Description:
        A wall clock the test moves: each call returns the time, and pose / sleep advance it.

    Initialization of the Class:
        Example:
            Initialization:
                # Assignment of the variables.
                clock = _Clock_Cls()

            Features:
                # Functions of the class.
                clock(); clock.Sleep(0.1)
    """

    def __init__(self):
        self.Now = 100.0

    def __call__(self):
        return self.Now

    def Sleep(self, seconds):
        self.Now += seconds


def _Timeline(steps=4, length=500):
    return np.repeat(np.arange(steps), length)


def test_the_overlay_is_never_set_under_the_viewer_lock():
    """
    Description:
        The deadlock of show_program.py (2026-10-24): set_texts was called inside `with viewer.lock():`. The loop holds the
        lock only for posing and calls set_texts and sync outside it; the fake viewer fails if either runs under the lock.
    """

    viewer, clock = _Fake_Viewer(120), _Clock_Cls()
    posed = []

    def Pose(i):
        assert viewer.Lock.locked()
        posed.append(i)

    stats = Run_Viewer_Loop(viewer, Playback_Cls(_Timeline(), 0.004), Pose, lambda i: ('t %d' % i, 'line', i // 500), clock=clock,
                            sleep=clock.Sleep)
    assert len(stats.Work) == 120 and posed and stats.Overlay_Updates >= 1
    assert sum(call[0] == 'sync' for call in viewer.Calls) == 120


def test_the_playback_follows_the_wall_clock_and_skips_samples_on_slow_frames():
    """
    Description:
        Running, the sample is the wall clock's (dt 4 ms): a frame period of 1/60 s advances 4 samples; a pose that takes
        50 ms skips 12-13 samples per frame instead of slowing the playback; paused, the sample stays and sync still runs.
    """

    viewer, clock = _Fake_Viewer(30), _Clock_Cls()
    playback = Playback_Cls(_Timeline(), 0.004)
    shown = []
    Run_Viewer_Loop(viewer, playback, shown.append, lambda i: ('', '', 0), clock=clock, sleep=clock.Sleep)
    steps = np.diff(shown)
    assert set(steps.tolist()) <= {4, 5} and abs(shown[-1] - 29 * CONST_FRAME_PERIOD / 0.004) <= 1

    viewer, clock = _Fake_Viewer(10), _Clock_Cls()
    playback, shown = Playback_Cls(_Timeline(), 0.004), []

    def Slow(i):
        shown.append(i)
        clock.Now += 0.050

    Run_Viewer_Loop(viewer, playback, Slow, lambda i: ('', '', 0), clock=clock, sleep=clock.Sleep)
    assert set(np.diff(shown).tolist()) <= {12, 13}

    viewer, clock = _Fake_Viewer(20), _Clock_Cls()
    playback, shown = Playback_Cls(_Timeline(), 0.004, sample=700, paused=True), []
    Run_Viewer_Loop(viewer, playback, shown.append, lambda i: ('', '', 0), clock=clock, sleep=clock.Sleep)
    assert shown == [700] and sum(call[0] == 'sync' for call in viewer.Calls) == 20


def test_keys_arriving_mid_frame_apply_at_the_next_frame():
    """
    Description:
        A key's action arrives on the viewer's thread at any time; it is queued and applied at the start of the next
        frame: next / previous / end jump to the step's bounds, pause stops and resumes counting from the shown sample,
        resuming at the end starts again, and several keys in one frame apply in order.
    """

    playback = Playback_Cls(_Timeline(), 0.004, sample=600, paused=True)
    assert playback.Advance(0.0) == 600
    playback.Request('next')
    assert playback.Sample == 600
    assert playback.Advance(0.0) == 1000
    playback.Request('end')
    assert playback.Advance(0.0) == 1499
    playback.Request('previous')
    assert playback.Advance(0.0) == 1000
    playback.Request('previous')
    assert playback.Advance(0.0) == 500
    playback.Request('previous'), playback.Request('previous'), playback.Request('next')
    assert playback.Advance(0.0) == 0
    playback.Request('pause')
    assert playback.Advance(10.0) == 0 and not playback.Paused
    assert playback.Advance(10.4) == 100
    playback.Request('pause')
    assert playback.Advance(20.0) == 100 and playback.Paused
    playback.Request('previous'), playback.Request('previous'), playback.Request('end')
    assert playback.Advance(20.0) == 1999
    playback.Request('pause')
    assert playback.Advance(30.0) == 0 and not playback.Paused
    with pytest.raises(ValueError):
        playback.Request('dance')

    viewer, clock = _Fake_Viewer(3), _Clock_Cls()
    playback, shown = Playback_Cls(_Timeline(), 0.004, sample=10, paused=True), []

    def Pose(i):
        shown.append(i)
        playback.Request('next')

    Run_Viewer_Loop(viewer, playback, Pose, lambda i: ('', '', 0), clock=clock, sleep=clock.Sleep)
    assert shown == [10, 500, 1000]


def test_the_overlay_updates_at_most_four_times_a_second_and_at_once_on_a_segment_change():
    """
    Description:
        The overlay's text changes every frame (the time); it is set at most every CONST_OVERLAY_PERIOD (0.25 s), except at
        once when the segment changes, and never when the text is unchanged (paused). The segment-change callback runs once
        per segment.
    """

    viewer, clock = _Fake_Viewer(180), _Clock_Cls()
    segments = []
    Run_Viewer_Loop(viewer, Playback_Cls(_Timeline(), 0.004), lambda i: None, lambda i: ('t %d' % i, 'line', i // 250),
                    lambda title, comment: segments.append(title), clock=clock, sleep=clock.Sleep)
    texts = [call for call in viewer.Calls if call[0] == 'set_texts']
    seconds = 180 * CONST_FRAME_PERIOD
    changes = int(179 * CONST_FRAME_PERIOD / 0.004) // 250 + 1
    assert len(segments) == changes
    assert changes <= len(texts) <= changes + int(seconds / CONST_OVERLAY_PERIOD) + 1
    shown = [int(text[1].split()[1]) // 250 for text in texts]
    assert sorted(set(shown)) == list(range(changes))

    viewer, clock = _Fake_Viewer(60), _Clock_Cls()
    Run_Viewer_Loop(viewer, Playback_Cls(_Timeline(), 0.004, paused=True), lambda i: None, lambda i: ('t %d' % i, 'line', 0),
                    clock=clock, sleep=clock.Sleep)
    assert sum(call[0] == 'set_texts' for call in viewer.Calls) == 1


def test_a_stalled_loop_is_reported_by_the_watchdog():
    """
    Description:
        --watchdog: a loop that stops beating is reported once (on_stall); without on_stall, every thread's traceback goes
        to stderr and the process exits CONST_WATCHDOG_EXIT (a subprocess whose loop blocks on a lock it already holds).
    """

    stalled = threading.Event()
    watchdog = Watchdog_Cls(0.2, on_stall=stalled.set)
    for _ in range(5):
        watchdog.Beat()
        assert not stalled.wait(0.05)
    assert stalled.wait(2.0)
    watchdog.Stop()

    src = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src')
    code = ('import sys, threading; sys.path.append(%r)\n'
            'from Render.Core import Watchdog_Cls\n'
            'w = Watchdog_Cls(0.3)\n'
            'lock = threading.Lock()\n'
            'def Frame():\n'
            '    with lock:\n'
            '        lock.acquire()\n'
            'Frame()\n' % src)
    result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=60)
    assert result.returncode == CONST_WATCHDOG_EXIT
    assert 'has not advanced for 0.3 s' in result.stderr and 'in Frame' in result.stderr and 'Thread' in result.stderr
