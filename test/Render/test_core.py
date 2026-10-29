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
from Render.Core import (Render_Error_Cls, Playback_Cls, Run_Viewer_Loop, Watchdog_Cls, CONST_FRAME_PERIOD,
                         CONST_OVERLAY_PERIOD, CONST_WATCHDOG_EXIT)


@pytest.fixture(scope='module')
def state(cell_model):
    data = mujoco.MjData(cell_model)
    mujoco.mj_forward(cell_model, data)
    return cell_model, data


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


def test_a_physics_playback_prepares_outside_the_lock_and_runs_at_its_speed():
    """
    Description:
        F11 (show_physics.py): prepare is called every frame before the lock is taken, the pose under it every frame (the run's
        state moves even when the sample does not); at speed 0.25 a 1/60 s frame advances one sample (4 ms) where real time
        advances four; 'frame' steps one sample and pauses; a speed of 0 is refused.
    """

    viewer, clock = _Fake_Viewer(40), _Clock_Cls()
    prepared, posed = [], []

    def Prepare(i):
        assert not viewer.Lock.locked()
        prepared.append(i)

    def Pose(i):
        assert viewer.Lock.locked()
        posed.append(i)

    playback = Playback_Cls(_Timeline(), 0.004, speed=0.25)
    Run_Viewer_Loop(viewer, playback, Pose, lambda i: ('', '', 0), clock=clock, sleep=clock.Sleep, prepare=Prepare)
    assert len(prepared) == len(posed) == 40 and prepared == posed
    assert set(np.diff(prepared).tolist()) <= {1, 2} and abs(prepared[-1] - 39 * CONST_FRAME_PERIOD * 0.25 / 0.004) <= 1
    playback = Playback_Cls(_Timeline(), 0.004, sample=10)
    playback.Request('frame')
    assert playback.Advance(0.0) == 11 and playback.Paused and playback.Advance(1.0) == 11
    with pytest.raises(ValueError, match='speed must be > 0'):
        Playback_Cls(_Timeline(), 0.004, speed=0.0)
