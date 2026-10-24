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
import sys
# Time (Time access)
import time
# Threading (Thread-based parallelism)
import threading
# Collections (Container datatypes)
import collections
# Faulthandler (Dump the Python traceback)
import faulthandler
# Dataclasses (Data Classes)
from dataclasses import dataclass, field
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
# The viewer loop: one frame per 1/60 s (the posing and the sync must fit into it; a slower frame skips samples by the wall
# clock), the overlay text at most every 0.25 s unless the segment changes (mujoco 3.16's set_texts waits for the render
# thread, design-decisions).
#   Unit [float] (s)
CONST_FRAME_PERIOD, CONST_OVERLAY_PERIOD = 1.0 / 60.0, 0.25
# The playback's key actions.
#   Unit [Tuple<string>]
CONST_PLAYBACK_ACTIONS = ('pause', 'next', 'previous', 'end')
# The exit code of a stalled viewer loop (--watchdog).
#   Unit [int]
CONST_WATCHDOG_EXIT = 3


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



class Playback_Cls(object):
    """
    Description:
        The playback state of a sampled timeline divided into steps: the sample shown, running or paused. A running
        playback follows the wall clock, so a slow frame skips samples instead of slowing the playback. The keys' actions
        arrive on the viewer's thread; they are queued and applied by the loop at the start of a frame (Advance), never
        in the middle of one.

    Initialization of the Class:
        Args:
            (1) step_index [Vector<int>]: The step of every sample (0, 1, ..., without gaps, in order).
            (2) dt [float]: The time between two samples (s).
            (3) sample [int]: The first sample shown.
            (4) paused [bool]: Start paused.

        Example:
            Initialization:
                # Assignment of the variables.
                playback = Playback_Cls(trajectory.Step_Index, CONST_DT)

            Features:
                # Properties of the class.
                playback.Sample, playback.Paused

                # Functions of the class.
                playback.Request('next'); playback.Advance(time.perf_counter())
    """

    def __init__(self, step_index: np.ndarray, dt: float, sample: int = 0, paused: bool = False) -> None:
        index = np.asarray(step_index)
        self.Step_Index, self.Dt, self.N = index, float(dt), len(index)
        self.Starts = [int(np.flatnonzero(index == k)[0]) for k in range(int(index.max()) + 1)]
        self.Ends = [int(np.flatnonzero(index == k)[-1]) for k in range(int(index.max()) + 1)]
        self.Sample, self.Paused = int(sample), bool(paused)
        self._Requests = collections.deque()
        self._Reference: tp.Optional[tp.Tuple[float, int]] = None

    def Request(self, action: str) -> None:
        """
        Description:
            Queue a key's action (any thread): 'pause' (pause / resume; resuming at the end starts again), 'next' (the
            next step's start), 'previous' (the current step's start, or the previous step's when already there), 'end'
            (the current step's end).

        Args:
            (1) action [string]: One of CONST_PLAYBACK_ACTIONS.
        """

        if action not in CONST_PLAYBACK_ACTIONS:
            raise ValueError('%r is not a playback action (%s)' % (action, ', '.join(CONST_PLAYBACK_ACTIONS)))
        self._Requests.append(action)

    def _Apply(self, action: str) -> None:
        """
        Description:
            Apply one queued action; a jump makes a running playback count on from the new sample.

        Args:
            (1) action [string]: The action.
        """

        k = int(self.Step_Index[self.Sample])
        if action == 'pause':
            self.Paused = not self.Paused
            if not self.Paused and self.Sample == self.N - 1:
                self.Sample = 0
        elif action == 'next':
            self.Sample = self.Starts[(k + 1) % len(self.Starts)]
        elif action == 'previous':
            self.Sample = self.Starts[k] if self.Sample > self.Starts[k] else self.Starts[(k - 1) % len(self.Starts)]
        else:
            self.Sample = self.Ends[k]
        self._Reference = None

    def Advance(self, now: float) -> int:
        """
        Description:
            The sample of this frame: the queued actions applied in order, then, while running, the sample the wall clock
            has reached since the playback started counting (pausing at the last sample).

        Args:
            (1) now [float]: The wall-clock time (s).

        Returns:
            (1) parameter [int]: The sample to show.
        """

        while self._Requests:
            self._Apply(self._Requests.popleft())
        if self.Paused:
            self._Reference = None
            return self.Sample
        if self._Reference is None:
            self._Reference = (now, self.Sample)
        start_time, start_sample = self._Reference
        self.Sample = min(start_sample + int((now - start_time) / self.Dt), self.N - 1)
        if self.Sample == self.N - 1:
            self.Paused, self._Reference = True, None
        return self.Sample


class Watchdog_Cls(object):
    """
    Description:
        A debugging watchdog for a loop: when Beat is not called for longer than its time, every thread's traceback is
        written to stderr (faulthandler) and the process exits with CONST_WATCHDOG_EXIT (or on_stall is called instead).

    Initialization of the Class:
        Args:
            (1) seconds [float]: The longest time without a beat (s).
            (2) on_stall [function]: Called once on a stall instead of the dump and exit (tests), or None.

        Example:
            Initialization:
                # Assignment of the variables.
                watchdog = Watchdog_Cls(5.0)

            Features:
                # Functions of the class.
                watchdog.Beat(); watchdog.Stop()
    """

    def __init__(self, seconds: float, on_stall: tp.Optional[tp.Callable[[], None]] = None) -> None:
        self.Seconds, self._On_Stall = float(seconds), on_stall
        self._Last, self._Done = time.monotonic(), threading.Event()
        self._Thread = threading.Thread(target=self._Watch, name='watchdog', daemon=True)
        self._Thread.start()

    def Beat(self) -> None:
        """
        Description:
            The loop advanced.
        """

        self._Last = time.monotonic()

    def Stop(self) -> None:
        """
        Description:
            End the watching.
        """

        self._Done.set()

    def _Watch(self) -> None:
        """
        Description:
            The watching thread: wakes a few times per watch time and acts once on a stall.
        """

        while not self._Done.wait(self.Seconds / 4.0):
            if time.monotonic() - self._Last > self.Seconds:
                if self._On_Stall is not None:
                    self._On_Stall()
                    return
                sys.stderr.write('[ERROR] The viewer loop has not advanced for %.1f s (--watchdog); every thread:\n' % self.Seconds)
                faulthandler.dump_traceback(file=sys.stderr, all_threads=True)
                sys.stderr.flush()
                os._exit(CONST_WATCHDOG_EXIT)


@dataclass
class Frame_Stats_Str:
    """
    Description:
        The work of a viewer loop's frames (the posing, the overlay and the sync; the sleep excluded).
    """

    # The frame's work times.
    #   Unit [List<float>] (s)
    Work: tp.List[float] = field(default_factory=list)
    # The number of overlay updates (set_texts calls).
    #   Unit [int]
    Overlay_Updates: int = 0

    def Summary(self) -> str:
        """
        Description:
            One line: the frames, the mean and the largest work per frame, the overlay updates.

        Returns:
            (1) parameter [string]: The line.
        """

        work = np.asarray(self.Work if self.Work else [0.0]) * 1000.0
        return ('%d frames: %.2f ms mean, %.2f ms max work per frame (budget %.1f ms); %d overlay updates'
                % (len(self.Work), work.mean(), work.max(), CONST_FRAME_PERIOD * 1000.0, self.Overlay_Updates))


def Run_Viewer_Loop(viewer: tp.Any, playback: Playback_Cls, pose: tp.Callable[[int], None],
                    describe: tp.Callable[[int], tp.Tuple[str, str, int]], on_segment: tp.Optional[tp.Callable[[str, str], None]] = None,
                    watchdog: tp.Optional[Watchdog_Cls] = None, frame_period: float = CONST_FRAME_PERIOD,
                    overlay_period: float = CONST_OVERLAY_PERIOD, clock: tp.Callable[[], float] = time.perf_counter,
                    sleep: tp.Callable[[float], None] = time.sleep) -> Frame_Stats_Str:
    """
    Description:
        Play a timeline in a passive viewer until it is closed, by the rules of mujoco 3.16's viewer: viewer.lock() is held
        only while the data is posed (pose); set_texts is called outside it (it waits for the render thread, which needs
        the same lock: called under it, both threads block), only when the text changes - at once on a segment change,
        otherwise at most every overlay_period; sync every frame, paused too, so the window always answers; the sleep fills
        the frame period, and a frame slower than it skips samples (Playback_Cls follows the wall clock).

    Args:
        (1) viewer [mujoco.viewer.Handle]: The passive viewer (is_running, lock, set_texts, sync).
        (2) playback [Playback_Cls(object)]: The playback state; its key actions are queued by the viewer's key callback.
        (3) pose [function]: Poses the viewer's data at a sample (called under the lock).
        (4) describe [function]: The overlay's two text columns and the segment of a sample.
        (5) on_segment [function]: Called with the text when the segment changes (the terminal line), or None.
        (6) watchdog [Watchdog_Cls(object)]: Beaten every frame, or None.
        (7 - 8) frame_period, overlay_period [float]: The frame period and the shortest time between overlay updates (s).
        (9 - 10) clock, sleep [function]: The wall clock (s) and the sleep (tests replace them).

    Returns:
        (1) parameter [Frame_Stats_Str(object)]: The frames' work.
    """

    stats, shown, text, segment, text_time = Frame_Stats_Str(), -1, None, None, -float('inf')
    while viewer.is_running():
        start = clock()
        sample = playback.Advance(start)
        if sample != shown:
            with viewer.lock():
                pose(sample)
            shown = sample
        title, comment, key = describe(sample)
        if (title, comment) != text and (key != segment or start - text_time >= overlay_period):
            viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT, title, comment))
            stats.Overlay_Updates += 1
            if key != segment and on_segment is not None:
                on_segment(title, comment)
            text, segment, text_time = (title, comment), key, start
        viewer.sync()
        work = clock() - start
        stats.Work.append(work)
        if watchdog is not None:
            watchdog.Beat()
        sleep(max(0.0, frame_period - work))
    return stats
