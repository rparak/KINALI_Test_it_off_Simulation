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
# File Name: ../Example/Report/make_media.py
## =========================================================================== ##

# System (Default)
import sys
# OS (Operating system interfaces)
import os
# The repository root, built from this file's location, so the script runs from any working directory.
#   Unit [string]
CONST_REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
#   Add access if it is not in the system path.
if os.path.join(CONST_REPO_ROOT, 'src') not in sys.path:
    sys.path.append(os.path.join(CONST_REPO_ROOT, 'src'))
# Argparse (Command-line parsing)
import argparse
# Functools (Higher-order functions: the scripts loaded once)
import functools
# Importlib (Import another script as a module)
import importlib.util
# Multiprocessing (The programs' physics runs in parallel)
import multiprocessing
# Time (Wall-clock timing)
import time
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Pillow (The legends, the scaled tiles) [pip3 install pillow]
from PIL import Image, ImageDraw, ImageFont
# Custom Lib.:
#   ../Configuration/Core - the configuration, the logs, the folders
from Configuration.Core import Configure_MuJoCo_Log, Load_Cell_Configuration, Read_PCB_Meta, Script_Log_Cls, CONST_REPORTS
#   ../Configuration/Const - the repository root
from Configuration.Const import CONST_ROOT
#   ../Configuration/Program_Parser - the points and their scenes
from Configuration.Program_Parser import Load_Constants, Load_Simulation
#   ../Scene/Core - the cell
from Scene.Core import Build_Cell_Model
#   ../Collision/Core - the check model (the table top as boxes), the loader's and MG06's bodies
from Collision.Core import Build_Planning_Model, CONST_LOADER_BODIES, CONST_MG06_UPPER_BODIES
#   ../Kinematics/Core, ../Keypoints/Core - posing the cell at a point
from Kinematics.Core import Robot_Kinematics_Cls
from Keypoints.Core import Pose_Cell
#   ../Simulation/Core - the drive
from Simulation.Core import Trajectory_Drive_Cls, Simulation_Error_Cls
#   ../Trajectory/Core - the sample time
from Trajectory.Core import CONST_DT
#   ../Report/Core, ../Report/Media, ../Report/Style - the recording, the frames, the encoders, the names and the palette
from Report.Core import Record_Physics
from Report.Media import (Frame_Renderer_Cls, Label_Frame, Frame_Samples, Video_Writer_Cls, Write_Gif, Write_Png, Contact_Sheet, Collision_Overlay,
                          Media_Error_Cls, CONST_VIEWS)
from Report.Style import CONST_PROGRAM_NAMES, CONST_OKABE_ITO

"""
Description:
    Initialization of constants.
"""
# The programs and their file stems.
#   Unit [Dict<string, string>]
CONST_STEMS = {'taught': 'Default', 'optimized': 'Optimized'}
# The trajectory files of the programs in a --trajectories folder (build_program.py's names).
#   Unit [Dict<string, string>]
CONST_TRAJECTORY_FILES = {'taught': 'Program.npz', 'optimized': 'Program_Optimized.npz'}
# The folders: the full media (git-ignored) and the README's (committed).
#   Unit [string]
CONST_OUT = os.path.join(CONST_REPORTS, 'Comparison')
CONST_MEDIA = os.path.join(CONST_ROOT, 'Media')
# The videos (owner, 2026-10-25): real time, 60 frames per second; the main ones in Overview_Right at 1080p and 1440p; the
# mosaic of four views, each 1280 x 720 (1440p); the Default and the Optimized program side by side, each 1280 x 720.
#   Unit [float] (1/s), [Dict<string, Tuple<int>>] (px), [string], [List<string>], [Tuple<int>] (px)
CONST_FPS = 60.0
CONST_RESOLUTIONS = {'1080p': (1920, 1080), '1440p': (2560, 1440)}
CONST_MAIN_VIEW = 'Overview_Right'
CONST_MOSAIC_VIEWS = ['Overview_Right', 'Loader_Handover', 'MG06_Insert', 'MG06_Fork']
CONST_TILE_SIZE = (1280, 720)
# The events (owner): (name, kind, which, the close view): kind an event of the trajectory ('vacuum_on', ...) and its
# occurrence, or 'step:<name>' and the fraction of that step; the clips cover CONST_CLIP seconds before and after (1080p).
#   Unit [List<Tuple>], [float] (s)
CONST_EVENTS = [('pick', 'vacuum_on', 0, 'Input_Crate'), ('handover', 'gripper_clamp', 0, 'Loader_Handover'),
                ('insert', 'gripper_open', 0, 'MG06_Insert'), ('fork_close', 'step:Close_MG06', 0.5, 'MG06_Fork'),
                ('fork_open', 'step:Open_MG06', 0.5, 'MG06_Fork'), ('fetch', 'gripper_clamp', 1, 'MG06_Insert'),
                ('take', 'vacuum_on', 1, 'Loader_Handover'), ('place', 'vacuum_off', 1, 'Output_Crate')]
CONST_CLIP = 3.0
# The images (owner): every view at these moments (as CONST_EVENTS), 2560 x 1440 PNG.
#   Unit [List<Tuple>], [Tuple<int>] (px)
CONST_MOMENTS = [('pick', 'vacuum_on', 0), ('handover', 'gripper_clamp', 0), ('insert', 'gripper_open', 0), ('MG06_closed', 'step:Close_MG06', 1.0),
                 ('test', 'step:Test', 0.5), ('take', 'vacuum_on', 1), ('place', 'vacuum_off', 1)]
CONST_IMAGE_SIZE = (2560, 1440)
# The README's curated images (owner: at most 1.5 MB each) from the Optimized program: (file, view, moment); the collision
# overlays (file, view, moment) and the collision-only one.
#   Unit [List<Tuple<string>>], [int] (bytes)
CONST_CURATED = [('View_Overview_Right', 'Overview_Right', 'handover'), ('View_Input_Crate', 'Input_Crate', 'pick'),
                 ('View_Output_Crate', 'Output_Crate', 'place'), ('View_Loader_Handover', 'Loader_Handover', 'handover'),
                 ('View_MG06_Insert', 'MG06_Insert', 'insert'), ('View_MG06_Fork', 'MG06_Fork', 'MG06_closed')]
CONST_OVERLAYS = [('Collision_Overview_Right', 'Overview_Right', 'handover', True),
                  ('Collision_Loader_Handover', 'Loader_Handover', 'handover', True),
                  ('Collision_MG06_Fork', 'MG06_Fork', 'MG06_closed', False), ('Collision_Input_Crate', 'Input_Crate', 'pick', False)]
CONST_MEDIA_BYTES = 1500 * 1000
# The GIF previews (owner): 960 px wide, 4 x speed, 15 frames per second, at most 10 MB.
#   Unit [int] (px), [float] (-), [float] (1/s), [int] (bytes)
CONST_GIF_WIDTH, CONST_GIF_SPEED, CONST_GIF_FPS, CONST_GIF_BYTES = 960, 4.0, 15.0, 10 * 1000 * 1000
# The key points' contact sheet: a tile's size and the columns.
#   Unit [Tuple<int>] (px), [int] (-)
CONST_TILE, CONST_SHEET_COLUMNS = (640, 360), 5
# The collision categories (owner, 2026-10-25), the figure palette (Okabe-Ito): (name, colour, which body / geom).
#   Unit [List<Tuple<string, string, function>>]
CONST_CATEGORIES = [('robot', CONST_OKABE_ITO['blue'], lambda b, g: b.startswith('aubo/') and not b.startswith('aubo/tool')),
                    ('tool: VGC10 + fork + cups', CONST_OKABE_ITO['green'], lambda b, g: b.startswith('aubo/tool')),
                    ('MG06', CONST_OKABE_ITO['purple'], lambda b, g: b == 'mg06_base' or b in CONST_MG06_UPPER_BODIES),
                    ('loader: axes, carriages, jaws', CONST_OKABE_ITO['vermillion'], lambda b, g: b in CONST_LOADER_BODIES or b == 'loader_base'),
                    ('crates', CONST_OKABE_ITO['orange'], lambda b, g: g.startswith('crate_')),
                    ('table boxes (planning only)', CONST_OKABE_ITO['yellow'], lambda b, g: g.startswith('table_')),
                    ('workstation: frame, pedestal', CONST_OKABE_ITO['grey'], lambda b, g: b in ('workstation', 'world')),
                    ('PCB', CONST_OKABE_ITO['sky'], lambda b, g: b == 'pcb')]


@functools.lru_cache(maxsize=None)
def _Script(name: str) -> tp.Any:
    """
    Description:
        Another Example script as a module, loaded once (run_program.py: the program loading and its stale checks).

    Args:
        (1) name [string]: The script's path below Example/.

    Returns:
        (1) parameter [module]: The script's module.
    """

    spec = importlib.util.spec_from_file_location(os.path.splitext(os.path.basename(name))[0], os.path.join(CONST_REPO_ROOT, 'Example', name))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _Shown(path: str) -> str:
    """
    Description:
        A file for a printed line: relative to the repository when inside it, else absolute.

    Args:
        (1) path [string]: The file.

    Returns:
        (1) parameter [string]: The text.
    """

    path = os.path.abspath(path)
    return os.path.relpath(path, CONST_ROOT) if path.startswith(CONST_ROOT + os.sep) else path


def Load(version: str, trajectories: tp.Optional[str]) -> tp.Tuple[tp.Any, ...]:
    """
    Description:
        A program's run inputs (run_program.py's Load_Run: refused when stale), its trajectory from a folder or the default.

    Args:
        (1) version [string]: 'taught' or 'optimized'.
        (2) trajectories [string]: The folder of Program.npz and Program_Optimized.npz, or None (Data/Generated/Trajectories).

    Returns:
        (1) parameter [Tuple]: Load_Run's (cell, pcb_meta, program, trajectory, free_steps, model, trajectory_path).
    """

    path = os.path.join(trajectories, CONST_TRAJECTORY_FILES[version]) if trajectories else None
    return _Script('Simulation/run_program.py').Load_Run(version, path)


def Record(job: tp.Tuple[str, tp.Optional[str]]) -> tp.Tuple[str, tp.Any, tp.Any, float]:
    """
    Description:
        A program's physics run recorded every 4 ms (a pool job; Report.Core.Record_Physics).

    Args:
        (1) job [Tuple<string, string>]: ('taught' or 'optimized', the trajectories' folder or None).

    Returns:
        (1) parameter [Tuple]: (version, the trajectory, the recording, the run's wall time (s)).
    """

    version, trajectories = job
    began = time.perf_counter()
    cell, meta, program, tr, free, model, _ = Load(version, trajectories)
    _, record = Record_Physics(model, Trajectory_Drive_Cls(tr, program, free), meta, cell)
    return version, tr, record, time.perf_counter() - began


def Moment(trajectory: tp.Any, kind: str, which: tp.Union[int, float]) -> float:
    """
    Description:
        The time of a moment: the which-th event of a kind ('vacuum_on', 'gripper_clamp', ...), or for 'step:<name>' the point a
        fraction which through the step <S...>_<name>.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2) kind [string]: The event or 'step:<name>'.
        (3) which [int or float]: The occurrence, or the fraction (0 to 1).

    Returns:
        (1) parameter [float]: The time (s).
    """

    tr = trajectory
    if kind.startswith('step:'):
        index = next(k for k, n in enumerate(tr.Step_Names) if n.split('_', 1)[-1] == kind[5:])
        samples = np.flatnonzero(tr.Step_Index == index)
        return float(tr.T[samples[int(round(which * (len(samples) - 1)))]])
    return float([t for t, e in tr.Events if e == kind][int(which)])


def Frame_Lines(version: str, trajectory: tp.Any, i: int, view: tp.Optional[str] = None) -> tp.List[str]:
    """
    Description:
        A frame's label: the program, its step, the time (and the view), then the line running.

    Args:
        (1) version [string]: The program.
        (2) trajectory [Trajectory_Str(object)]: The trajectory.
        (3) i [int]: The sample.
        (4) view [string]: The view's name, or None.

    Returns:
        (1) parameter [List<string>]: The two lines.
    """

    tr = trajectory
    i = min(i, len(tr.T) - 1)
    segment = min(int(np.searchsorted(tr.Segment_End, i, side='right')), len(tr.Segment_Command) - 1)
    head = '%s  |  %s  |  t = %.2f s / %.2f s (physics)' % (CONST_PROGRAM_NAMES[version], tr.Step_Names[tr.Step_Index[i]], tr.T[i], tr.T[-1])
    return [head + ('  |  %s' % view if view else ''), tr.Segment_Command[segment]]


class Poser_Cls(object):
    """
    Description:
        The cell posed at a recorded physics state, and frames of it from named views (one renderer per view and size, kept).

    Initialization of the Class:
        Args:
            (1) model [mujoco.MjModel]: The cell (Scene.Core) or the check model.

        Example:
            Initialization:
                Cls = Poser_Cls(model)

            Features:
                # Functions of the class.
                Cls.Frame(qpos, 'Overview_Right', (1920, 1080)); Cls.Close()
    """

    def __init__(self, model: mujoco.MjModel) -> None:
        self.Model, self.Data = model, mujoco.MjData(model)
        self.__renderers: tp.Dict[tp.Tuple[str, int, int], Frame_Renderer_Cls] = {}

    def Pose(self, qpos: np.ndarray) -> None:
        """
        Description:
            Pose the model at positions.

        Args:
            (1) qpos [Vector<float> nq]: The positions.
        """

        self.Data.qpos[:] = qpos
        mujoco.mj_forward(self.Model, self.Data)

    def Renderer(self, view: str, size: tp.Tuple[int, int]) -> Frame_Renderer_Cls:
        """
        Description:
            The renderer of a view and a size (made once).

        Args:
            (1) view [string]: The view (Report.Media.CONST_VIEWS).
            (2) size [Tuple<int>]: Width, height (px).

        Returns:
            (1) parameter [Frame_Renderer_Cls(object)]: The renderer.
        """

        key = (view, size[0], size[1])
        if key not in self.__renderers:
            self.__renderers[key] = Frame_Renderer_Cls(self.Model, size[0], size[1], CONST_VIEWS[view])
        return self.__renderers[key]

    def Frame(self, qpos: np.ndarray, view: str, size: tp.Tuple[int, int]) -> np.ndarray:
        """
        Description:
            A frame of positions from a view.

        Args:
            (1) qpos [Vector<float> nq]: The positions.
            (2) view [string]: The view.
            (3) size [Tuple<int>]: Width, height (px).

        Returns:
            (1) parameter [Matrix<uint8> HxWx3]: The frame.
        """

        self.Pose(qpos)
        return self.Renderer(view, size).Render(self.Data)

    def Close(self) -> None:
        """
        Description:
            Release the renderers.
        """

        for renderer in self.__renderers.values():
            renderer.Close()


def Encode(path: str, size: tp.Tuple[int, int], frames: tp.Iterable[np.ndarray], note: tp.Callable[[str], None],
           keep: tp.Optional[int] = None) -> tp.List[np.ndarray]:
    """
    Description:
        A video from frames (Report.Media.Video_Writer_Cls, CONST_FPS) with its render and encode time and size printed; every
        keep-th frame returned (for a GIF).

    Args:
        (1) path [string]: The file.
        (2) size [Tuple<int>]: Width, height (px).
        (3) frames [Iterable<Matrix<uint8>>]: The frames (rendered as they are taken).
        (4) note [function]: Prints a line.
        (5) keep [int]: Return every keep-th frame, or None.

    Returns:
        (1) parameter [List<Matrix<uint8>>]: The kept frames.
    """

    writer = Video_Writer_Cls(path, size[0], size[1], CONST_FPS)
    kept, began, render, count = [], time.perf_counter(), 0.0, 0
    iterator = iter(frames)
    while True:
        t0 = time.perf_counter()
        frame = next(iterator, None)
        render += time.perf_counter() - t0
        if frame is None:
            break
        writer.Write(frame)
        if keep is not None and count % keep == 0:
            kept.append(frame)
        count += 1
    writer.Close()
    total = time.perf_counter() - began
    note('[INFO] %s: %d frames, %.1f s at %g fps, %dx%d; render and label %.1f s, encode beyond the render %.1f s, total %.1f s; %.1f MB'
         % (_Shown(path), count, count / CONST_FPS, CONST_FPS, size[0], size[1], render, total - render, total, os.path.getsize(path) / 1e6))
    return kept


def Videos(poser: Poser_Cls, runs: tp.Dict[str, tp.Tuple[tp.Any, tp.Any]], args: tp.Any, note: tp.Callable[[str], None]) -> None:
    """
    Description:
        The videos (owner, 2026-10-25): per program the main one in CONST_MAIN_VIEW at each resolution, the 2 x 2 mosaic of
        CONST_MOSAIC_VIEWS (1440p), the event clips (1080p, CONST_CLIP s around each event in its close view); the Default and the
        Optimized program side by side from one start, each with its label and clock (1440p wide); the GIF previews from the
        side by side and from the Optimized program's main video.

    Args:
        (1) poser [Poser_Cls(object)]: The cell.
        (2) runs [Dict<string, Tuple>]: Per program its (trajectory, recording).
        (3) args [Namespace]: The script's options.
        (4) note [function]: Prints a line.
    """

    every = int(round(CONST_FPS * CONST_GIF_SPEED / CONST_GIF_FPS))
    end = lambda record: float(record.T[-1]) if args.until is None else min(args.until, float(record.T[-1]))
    for version, (tr, record) in runs.items():
        samples = Frame_Samples(end(record), CONST_DT, CONST_FPS, 1.0)
        for k, name in enumerate([] if args.skip_videos else args.resolution):
            size = CONST_RESOLUTIONS[name]
            frames = (Label_Frame(poser.Frame(record.Qpos[i], CONST_MAIN_VIEW, size), Frame_Lines(version, tr, int(i))) for i in samples)
            kept = Encode(os.path.join(args.out, '%s_%s_%s.mp4' % (CONST_STEMS[version], CONST_MAIN_VIEW, name)), size, frames, note,
                          every if (version == 'optimized' and k == 0) else None)
            if kept:
                gif = os.path.join(args.media, 'Cycle_Optimized.gif')
                size_gif, colours, count = Write_Gif(kept, gif, CONST_GIF_FPS, CONST_GIF_WIDTH, CONST_GIF_BYTES)
                note('[INFO] %s: %d frames, %g x speed, %d colours, %.1f MB' % (_Shown(gif), count, CONST_GIF_SPEED, colours, size_gif / 1e6))
        if not args.skip_mosaic:
            w, h = CONST_TILE_SIZE

            def Mosaic(i: int) -> np.ndarray:
                """
                Description:
                    The 2 x 2 mosaic of a sample, each tile labelled with its view, the whole with the program's label.

                Args:
                    (1) i [int]: The sample.

                Returns:
                    (1) parameter [Matrix<uint8> 2Hx2Wx3]: The frame.
                """

                tiles = [Label_Frame(poser.Frame(record.Qpos[i], view, CONST_TILE_SIZE), [view.replace('_', ' ')], 1.0 / 30.0)
                         for view in CONST_MOSAIC_VIEWS]
                frame = np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])])
                return Label_Frame(frame, Frame_Lines(version, tr, i))

            Encode(os.path.join(args.out, '%s_Mosaic_1440p.mp4' % CONST_STEMS[version]), (2 * w, 2 * h), (Mosaic(int(i)) for i in samples), note)
        if not args.skip_clips:
            for name, kind, which, view in CONST_EVENTS:
                t = Moment(tr, kind, which)
                a, b = max(0.0, t - args.clip), min(float(record.T[-1]), t + args.clip)
                clip = np.minimum(np.round((a + np.arange(0.0, b - a + 1e-9, 1.0 / CONST_FPS)) / CONST_DT).astype(int), len(record.T) - 1)
                size = CONST_RESOLUTIONS['1080p']
                Encode(os.path.join(args.out, 'Clips', '%s_%s.mp4' % (CONST_STEMS[version], name)), size,
                       (Label_Frame(poser.Frame(record.Qpos[i], view, size), Frame_Lines(version, tr, int(i), view)) for i in clip), note)
    if not args.skip_side and len(runs) == 2:
        w, h = CONST_TILE_SIZE
        longest = max(end(record) for _, record in runs.values())

        def Side(t: float) -> np.ndarray:
            """
            Description:
                The two programs at one time, side by side; a program that has ended holds its last frame, labelled with its end.

            Args:
                (1) t [float]: The time (s).

            Returns:
                (1) parameter [Matrix<uint8> Hx2Wx3]: The frame.
            """

            halves = []
            for version, (tr, record) in runs.items():
                i = min(int(round(t / CONST_DT)), len(record.T) - 1)
                lines = Frame_Lines(version, tr, i)
                if t > float(record.T[-1]):
                    lines = ['%s  |  done at %.2f s (t = %.2f s)' % (CONST_PROGRAM_NAMES[version], record.T[-1], t), lines[1]]
                halves.append(Label_Frame(poser.Frame(record.Qpos[i], CONST_MAIN_VIEW, CONST_TILE_SIZE), lines, 1.0 / 26.0))
            return np.hstack(halves)

        times = np.arange(0.0, longest + 1e-9, 1.0 / CONST_FPS)
        kept = Encode(os.path.join(args.out, 'Default_vs_Optimized_Side_by_Side.mp4'), (2 * w, h), (Side(float(t)) for t in times), note,
                      every)
        gif = os.path.join(args.media, 'Cycle_Side_by_Side.gif')
        size_gif, colours, count = Write_Gif(kept, gif, CONST_GIF_FPS, CONST_GIF_WIDTH, CONST_GIF_BYTES)
        note('[INFO] %s: %d frames, %g x speed, %d colours, %.1f MB' % (_Shown(gif), count, CONST_GIF_SPEED, colours, size_gif / 1e6))


def View_Images(poser: Poser_Cls, runs: tp.Dict[str, tp.Tuple[tp.Any, tp.Any]], args: tp.Any, note: tp.Callable[[str], None]) -> None:
    """
    Description:
        Every view of args.views at every moment of CONST_MOMENTS for each program (2560 x 1440 PNG, labelled) into
        <out>/Views/; the curated README images (CONST_CURATED, the Optimized program, at most CONST_MEDIA_BYTES each) into
        args.media.

    Args:
        (1) poser [Poser_Cls(object)]: The cell.
        (2) runs [Dict<string, Tuple>]: Per program its (trajectory, recording).
        (3) args [Namespace]: The script's options.
        (4) note [function]: Prints a line.
    """

    began, count, total = time.perf_counter(), 0, 0
    for version, (tr, record) in runs.items():
        for moment, kind, which in CONST_MOMENTS:
            i = min(int(round(Moment(tr, kind, which) / CONST_DT)), len(record.T) - 1)
            for view in args.views:
                image = Label_Frame(poser.Frame(record.Qpos[i], view, CONST_IMAGE_SIZE), Frame_Lines(version, tr, i, view))
                size, _ = Write_Png(image, os.path.join(args.out, 'Views', '%s_%s_%s.png' % (CONST_STEMS[version], view, moment)))
                count, total = count + 1, total + size
    note('[INFO] %d view images in %s, %.0f MB, %.1f s' % (count, _Shown(os.path.join(args.out, 'Views')), total / 1e6, time.perf_counter() - began))
    if 'optimized' in runs:
        tr, record = runs['optimized']
        for name, view, moment in CONST_CURATED:
            kind, which = next((k, w) for m, k, w in CONST_MOMENTS if m == moment)
            i = min(int(round(Moment(tr, kind, which) / CONST_DT)), len(record.T) - 1)
            image = Label_Frame(poser.Frame(record.Qpos[i], view, CONST_IMAGE_SIZE), Frame_Lines('optimized', tr, i, view))
            path = os.path.join(args.media, name + '.png')
            size, width = Write_Png(image, path, CONST_MEDIA_BYTES)
            note('[INFO] %s: %d px wide, %.2f MB' % (_Shown(path), width, size / 1e6))


def Point_Images(args: tp.Any, note: tp.Callable[[str], None]) -> None:
    """
    Description:
        The cell posed at every point of Constants.yaml (its own scene: Keypoints.Core.Pose_Cell) from CONST_MAIN_VIEW, one image
        each into <out>/Points/, and their contact sheet into args.media (at most CONST_MEDIA_BYTES).

    Args:
        (1) args [Namespace]: The script's options.
        (2) note [function]: Prints a line.
    """

    began = time.perf_counter()
    cell, meta, constants = Load_Cell_Configuration(), Read_PCB_Meta(), Load_Constants()
    simulation = Load_Simulation(constants)
    model = Build_Cell_Model(cell)
    kinematics = Robot_Kinematics_Cls(model)
    renderer = Frame_Renderer_Cls(model, *CONST_RESOLUTIONS['1080p'], CONST_VIEWS[CONST_MAIN_VIEW])
    tiles, labels = [], []
    for name in constants.Points:
        Pose_Cell(kinematics, constants, simulation, cell, meta, name)
        image = renderer.Render(kinematics.Data)
        Write_Png(Label_Frame(image, [name, constants.Points[name].Description]), os.path.join(args.out, 'Points', name + '.png'))
        tiles.append(np.asarray(Image.fromarray(image).resize(CONST_TILE, Image.Resampling.LANCZOS)))
        labels.append(name)
    renderer.Close()
    path = os.path.join(args.media, 'Key_Points.png')
    size, width = Write_Png(Contact_Sheet(tiles, labels, CONST_SHEET_COLUMNS), path, CONST_MEDIA_BYTES)
    note('[INFO] %d point images in %s; %s: %d px wide, %.2f MB (%.1f s)' % (len(tiles), _Shown(os.path.join(args.out, 'Points')), _Shown(path),
                                                                           width, size / 1e6, time.perf_counter() - began))


def _Legend(image: np.ndarray, entries: tp.List[tp.Tuple[str, str]], title: str) -> np.ndarray:
    """
    Description:
        A legend in an image's top left corner: a title and a swatch with its name per category, on a dark band.

    Args:
        (1) image [Matrix<uint8> HxWx3]: The image.
        (2) entries [List<Tuple<string, string>>]: (name, colour '#RRGGBB').
        (3) title [string]: The title line.

    Returns:
        (1) parameter [Matrix<uint8> HxWx3]: The image with the legend.
    """

    picture = Image.fromarray(image).convert('RGBA')
    band = Image.new('RGBA', picture.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(band)
    size = max(12, image.shape[0] // 48)
    font = ImageFont.load_default(size=size)
    step = int(size * 1.45)
    width = max(int(draw.textlength(t, font=font)) for t in [title] + [n for n, _ in entries]) + 3 * size
    draw.rectangle([0, 0, width + size, (len(entries) + 1) * step + size], fill=(11, 11, 11, 185))
    draw.text((size // 2, size // 2), title, font=font, fill=(240, 239, 236, 255))
    for k, (name, colour) in enumerate(entries, start=1):
        y = size // 2 + k * step
        rgb = tuple(int(colour[j:j + 2], 16) for j in (1, 3, 5))
        draw.rectangle([size // 2, y + 2, size // 2 + size, y + size + 2], fill=rgb + (255,))
        draw.text((2 * size, y), name, font=font, fill=(240, 239, 236, 255))
    return np.asarray(Image.alpha_composite(picture, band).convert('RGB'))


def Collision_Images(runs: tp.Dict[str, tp.Tuple[tp.Any, tp.Any]], args: tp.Any, note: tp.Callable[[str], None]) -> None:
    """
    Description:
        The collision structure (owner, 2026-10-25) on the check model (the cell with the planning-only table boxes) at the
        Optimized program's moments: the visual cell with the collision geoms laid over it semi-transparently in their
        category's colour (Report.Media.Collision_Overlay: depth-tested in image space, no z-fighting) for CONST_OVERLAYS, and the
        collision geoms only (visuals hidden, each category's colour) from CONST_MAIN_VIEW; 2560 x 1440 PNG into <out>/Views/, the
        overlays marked for the README (at most CONST_MEDIA_BYTES) into args.media. The model's files are not changed: every
        colour is set on a render-only copy.

    Args:
        (1) runs [Dict<string, Tuple>]: Per program its (trajectory, recording).
        (2) args [Namespace]: The script's options.
        (3) note [function]: Prints a line.
    """

    model = Build_Planning_Model(Load_Cell_Configuration())
    tr, record = runs.get('optimized', next(iter(runs.values())))
    collidable = np.flatnonzero((model.geom_contype > 0) | (model.geom_conaffinity > 0))
    colours, counts = {}, {}
    for g in collidable:
        body, geom = model.body(model.geom_bodyid[g]).name, model.geom(g).name or ''
        for name, colour, owns in CONST_CATEGORIES:
            if owns(body, geom):
                colours[int(g)] = tuple(int(colour[j:j + 2], 16) for j in (1, 3, 5))
                counts[name] = counts.get(name, 0) + 1
                break
    entries = [(('%s (%d)' % (name, counts[name])), colour) for name, colour, _ in CONST_CATEGORIES if counts.get(name)]
    groups = sorted({int(model.geom_group[g]) for g in collidable})
    data = mujoco.MjData(model)
    w, h = CONST_IMAGE_SIZE
    for name, view, moment, curated in CONST_OVERLAYS:
        kind, which = next((k, x) for m, k, x in CONST_MOMENTS if m == moment)
        data.qpos[:] = record.Qpos[min(int(round(Moment(tr, kind, which) / CONST_DT)), len(record.T) - 1)]
        mujoco.mj_forward(model, data)
        visual = Frame_Renderer_Cls(model, w, h, CONST_VIEWS[view])
        shapes = Frame_Renderer_Cls(model, w, h, CONST_VIEWS[view], groups=groups)
        image = Collision_Overlay(visual.Render(data), visual.Depth(data), shapes.Geoms(data), shapes.Depth(data), colours)
        visual.Close()
        shapes.Close()
        title = 'Collision geoms over the visual cell (%s, %s); simulation model, not the real cell' % (view.replace('_', ' '), moment)
        image = _Legend(image, entries, title)
        size, _ = Write_Png(image, os.path.join(args.out, 'Views', name + '.png'))
        note('[INFO] %s: %.1f MB' % (_Shown(os.path.join(args.out, 'Views', name + '.png')), size / 1e6))
        if curated:
            path = os.path.join(args.media, name + '.png')
            size, width = Write_Png(image, path, CONST_MEDIA_BYTES)
            note('[INFO] %s: %d px wide, %.2f MB' % (_Shown(path), width, size / 1e6))
    only = Frame_Renderer_Cls(model, w, h, CONST_VIEWS[CONST_MAIN_VIEW], groups=groups, tone=False)
    for g, rgb in colours.items():
        only.Model.geom_rgba[g] = [c / 255.0 for c in rgb] + [1.0]
        only.Model.geom_matid[g] = -1
    kind, which = next((k, x) for m, k, x in CONST_MOMENTS if m == 'handover')
    data.qpos[:] = record.Qpos[min(int(round(Moment(tr, kind, which) / CONST_DT)), len(record.T) - 1)]
    mujoco.mj_forward(model, data)
    image = _Legend(only.Render(data), entries, 'Collision geoms only (groups %s); visual geoms hidden' % ', '.join(map(str, groups)))
    only.Close()
    size, _ = Write_Png(image, os.path.join(args.out, 'Views', 'Collision_Only_Overview_Right.png'))
    note('[INFO] %s: %.1f MB; categories: %s' % (_Shown(os.path.join(args.out, 'Views', 'Collision_Only_Overview_Right.png')), size / 1e6,
                                                 ', '.join('%s %d' % (n, c) for n, c in counts.items())))


def main():
    """
    Description:
        The comparison's media (FEATURES.md F14, F15; owner 2026-10-25), from the named views of Report.Media.CONST_VIEWS on a
        render-only copy of the cell: the videos (Videos: the main ones, the mosaics, the event clips, the side by side) and the
        GIF previews (Media/Cycle_Side_by_Side.gif, Media/Cycle_Optimized.gif); every view at the key moments
        (<out>/Views/) and the curated README images (Media/View_*.png); the cell at every point and their contact sheet
        (Media/Key_Points.png); the collision structure laid over the cell (Media/Collision_*.png). The trajectories are refused
        when stale (build them with build_program.py). Exit 0, 2 when a run or a medium cannot be made.

            .venv/bin/python Example/Report/make_media.py [--out DIR] [--media DIR] [--program taught optimized] [--resolution 1080p 1440p]
                [--trajectories DIR] [--until S] [--clip S] [--views V ...] [--skip-videos] [--skip-mosaic] [--skip-side] [--skip-clips]
                [--skip-images] [--skip-points] [--skip-collision]
    """

    parser = argparse.ArgumentParser(description='The comparison\'s media: videos, GIF previews, view images, key points, collision structure.')
    parser.add_argument('--out', default=CONST_OUT, help='the full media (default: %s)' % _Shown(CONST_OUT))
    parser.add_argument('--media', default=CONST_MEDIA, help='the README\'s previews and images (default: Media)')
    parser.add_argument('--program', nargs='+', default=list(CONST_STEMS), choices=list(CONST_STEMS), help='the programs (default: both)')
    parser.add_argument('--resolution', nargs='+', default=['1080p', '1440p'], choices=list(CONST_RESOLUTIONS), help='the main videos')
    parser.add_argument('--trajectories', help='the folder of Program.npz and Program_Optimized.npz (default: Data/Generated/Trajectories)')
    parser.add_argument('--until', type=float, help='end the videos at this time of the cycle (s; default: the whole cycle)')
    parser.add_argument('--clip', type=float, default=CONST_CLIP, help='the event clips\' half length (s; default %g)' % CONST_CLIP)
    parser.add_argument('--views', nargs='+', default=list(CONST_VIEWS), choices=list(CONST_VIEWS), help='the views of the images (default: all)')
    for name in ('videos', 'mosaic', 'side', 'clips', 'images', 'points', 'collision'):
        parser.add_argument('--skip-' + name, action='store_true', help='no %s' % ('main videos' if name == 'videos' else name))
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('make_media')
    note = lambda line: print(log.Line(line), flush=True)
    began = time.perf_counter()
    run_program = _Script('Simulation/run_program.py')
    try:
        runs = {}
        if not all((args.skip_videos, args.skip_mosaic, args.skip_side, args.skip_clips, args.skip_images, args.skip_collision)):
            with multiprocessing.get_context('spawn').Pool(len(args.program), initializer=Configure_MuJoCo_Log) as pool:
                for version, tr, record, seconds in pool.map(Record, [(v, args.trajectories) for v in args.program], chunksize=1):
                    note('[INFO] %s: physics run recorded in %.1f s' % (CONST_PROGRAM_NAMES[version], seconds))
                    runs[version] = (tr, record)
        poser = Poser_Cls(Build_Cell_Model(Load_Cell_Configuration()))
        if runs:
            Videos(poser, runs, args, note)
        if not args.skip_images:
            View_Images(poser, runs, args, note)
        poser.Close()
        if not args.skip_points:
            Point_Images(args, note)
        if not args.skip_collision:
            Collision_Images(runs, args, note)
    except (run_program.Run_Error_Cls, Simulation_Error_Cls, Media_Error_Cls) as error:
        note('[ERROR] %s' % error)
        log.Close()
        return 2
    note('[INFO] Done in %.1f s. Simulation results only: not validated for the real cell.' % (time.perf_counter() - began))
    log.Close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
