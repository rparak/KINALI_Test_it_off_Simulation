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
# Pillow (The collision image's legend) [pip3 install pillow]
from PIL import Image, ImageDraw, ImageFont
# Custom Lib.:
#   ../Configuration/Core - the configuration, the logs, the folders
from Configuration.Core import Configure_MuJoCo_Log, Load_Cell_Configuration, Read_PCB_Meta, Script_Log_Cls, CONST_GENERATED
#   ../Configuration/Const - the repository root
from Configuration.Const import CONST_ROOT
#   ../Configuration/Program_Parser - the points and their scenes
from Configuration.Program_Parser import Load_Constants, Load_Simulation
#   ../Scene/Core - the cell
from Scene.Core import Build_Cell_Model
#   ../Collision/Core - the check model (the table top as boxes)
from Collision.Core import Build_Planning_Model, CONST_LOADER_BODIES, CONST_MG06_UPPER_BODIES
#   ../Kinematics/Core, ../Keypoints/Core - posing the cell at a point
from Kinematics.Core import Robot_Kinematics_Cls
from Keypoints.Core import Pose_Cell
#   ../Program/Check - posing at a trajectory sample
from Program.Check import Pose_Sample
#   ../Simulation/Core - the drive
from Simulation.Core import Trajectory_Drive_Cls, Simulation_Error_Cls
#   ../Render/Core - the PNG writer
from Render.Core import Save_Image
#   ../Trajectory/Core - the sample time
from Trajectory.Core import CONST_DT
#   ../Report/Core, ../Report/Media - the recording, the frames, the encoders
from Report.Core import Record_Physics
from Report.Media import Frame_Renderer_Cls, Label_Frame, Frame_Samples, Video_Writer_Cls, Write_Gif, Contact_Sheet, Media_Error_Cls

"""
Description:
    Initialization of constants.
"""
# The programs, their titles on the frames and their file stems.
#   Unit [Dict<string, Tuple<string, string>>]
CONST_VERSIONS = {'taught': ('Taught program', 'Taught'), 'optimized': ('Optimized program', 'Optimized')}
# The trajectory files of the programs in a --trajectories folder (build_program.py's names).
#   Unit [Dict<string, string>]
CONST_TRAJECTORY_FILES = {'taught': 'Program.npz', 'optimized': 'Program_Optimized.npz'}
# The folders: the full media (git-ignored) and the README's (committed).
#   Unit [string]
CONST_OUT = os.path.join(CONST_GENERATED, 'Report')
CONST_MEDIA = os.path.join(CONST_ROOT, 'Media')
# The fixed camera of every medium: the point looked at, the distance, the azimuth and the elevation (front-left, above:
# MG06, the loader, both crates and the robot in view; chosen by rendering candidates, 2026-10-25).
#   Unit [Tuple<float>] (m, m, m, m, deg, deg)
CONST_CAMERA = (0.10, -0.34, 0.40, 2.0, 78.0, -34.0)
# The videos (owner, 2026-10-25): real time, 60 frames per second, 1920 x 1080 (and 2560 x 1440 on request).
#   Unit [float] (1/s), [Dict<string, Tuple<int>>] (px)
CONST_FPS = 60.0
CONST_RESOLUTIONS = {'1080p': (1920, 1080), '1440p': (2560, 1440)}
# The GIF previews (owner, 2026-10-25): 960 px wide, 4 x speed, 15 frames per second, at most 10 MB.
#   Unit [int] (px), [float] (-), [float] (1/s), [int] (bytes)
CONST_GIF_WIDTH, CONST_GIF_SPEED, CONST_GIF_FPS, CONST_GIF_BYTES = 960, 4.0, 15.0, 10 * 1000 * 1000
# The key points' contact sheet: a tile's size and the columns.
#   Unit [Tuple<int>] (px), [int] (-)
CONST_TILE, CONST_SHEET_COLUMNS = (640, 360), 5
# The collision image: a colour per part (RGBA) and its bodies; the size.
#   Unit [List<Tuple<string, Tuple<float>, function>>], [Tuple<int>] (px)
CONST_COLLISION_PARTS = [('robot', (0.16, 0.47, 0.84, 1.0), lambda b: b.startswith('aubo/') and not b.startswith('aubo/tool')),
                         ('tool: VGC10 + fork', (0.11, 0.69, 0.48, 1.0), lambda b: b.startswith('aubo/tool')),
                         ('MG06', (0.91, 0.48, 0.64, 1.0), lambda b: b == 'mg06_base' or b in CONST_MG06_UPPER_BODIES),
                         ('loader', (0.92, 0.41, 0.20, 1.0), lambda b: b in CONST_LOADER_BODIES or b == 'loader_base'),
                         ('workstation: crates, table boxes, frame', (0.93, 0.63, 0.0, 1.0), lambda b: b in ('workstation', 'world')),
                         ('PCB', (0.0, 0.51, 0.0, 1.0), lambda b: b == 'pcb')]
CONST_COLLISION_SIZE = (1920, 1080)


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


def Frame_Lines(title: str, trajectory: tp.Any, i: int) -> tp.List[str]:
    """
    Description:
        A frame's label: the program, its step and the time, then the line running.

    Args:
        (1) title [string]: The program's title.
        (2) trajectory [Trajectory_Str(object)]: The trajectory.
        (3) i [int]: The sample.

    Returns:
        (1) parameter [List<string>]: The two lines.
    """

    tr = trajectory
    segment = int(np.searchsorted(tr.Segment_End, i, side='right'))
    segment = min(segment, len(tr.Segment_Command) - 1)
    return ['%s  |  %s  |  t = %.2f s / %.2f s (physics)' % (title, tr.Step_Names[tr.Step_Index[i]], tr.T[i], tr.T[-1]),
            tr.Segment_Command[segment]]


def Videos(model: mujoco.MjModel, version: str, trajectory: tp.Any, record: tp.Any, resolutions: tp.List[str], out: str,
           gif: tp.Optional[str], until: tp.Optional[float], note: tp.Callable[[str], None]) -> None:
    """
    Description:
        A program's videos (H.264, CONST_FPS, real time, the fixed camera, the step label) at each resolution, and its GIF
        preview from the first resolution's frames (every frame that falls on the preview's time step, scaled to
        CONST_GIF_WIDTH). Reports the render and encode time and the size of each.

    Args:
        (1) model [mujoco.MjModel]: The cell.
        (2) version [string]: The program.
        (3) trajectory [Trajectory_Str(object)]: Its trajectory (the labels).
        (4) record [Physics_Record_Str(object)]: Its physics run.
        (5) resolutions [List<string>]: Keys of CONST_RESOLUTIONS.
        (6) out [string]: The videos' folder.
        (7) gif [string]: The preview's file, or None.
        (8) until [float]: The time the videos end at (s), or None for the whole run.
        (9) note [function]: Prints a line.
    """

    title, stem = CONST_VERSIONS[version]
    data = mujoco.MjData(model)
    samples = Frame_Samples(float(record.T[-1]) if until is None else min(until, float(record.T[-1])), CONST_DT, CONST_FPS, 1.0)
    every = int(round(CONST_FPS * CONST_GIF_SPEED / CONST_GIF_FPS))
    frames = []
    for k, name in enumerate(resolutions):
        width, height = CONST_RESOLUTIONS[name]
        path = os.path.join(out, '%s_Program_%s.mp4' % (stem, name))
        renderer = Frame_Renderer_Cls(model, width, height, CONST_CAMERA)
        writer = Video_Writer_Cls(path, width, height, CONST_FPS)
        render, began = 0.0, time.perf_counter()
        for f, i in enumerate(samples):
            t0 = time.perf_counter()
            data.qpos[:] = record.Qpos[i]
            mujoco.mj_forward(model, data)
            frame = Label_Frame(renderer.Render(data), Frame_Lines(title, trajectory, int(i)))
            render += time.perf_counter() - t0
            writer.Write(frame)
            if gif is not None and k == 0 and f % every == 0:
                frames.append(frame)
        writer.Close()
        renderer.Close()
        total = time.perf_counter() - began
        note('[INFO] %s: %d frames, %.1f s; render and label %.1f s, encode (beyond the render, ffmpeg in parallel) %.1f s, total %.1f s; %.1f MB'
             % (_Shown(path), len(samples), len(samples) / CONST_FPS, render, total - render, total,
                os.path.getsize(path) / 1e6))
    if gif is not None:
        t0 = time.perf_counter()
        size, colours, kept = Write_Gif(frames, gif, CONST_GIF_FPS, CONST_GIF_WIDTH, CONST_GIF_BYTES)
        note('[INFO] %s: %d frames (%d kept), %.0f px wide, %g x speed, %d colours, %.1f MB, %.1f s' % (
            _Shown(gif), len(frames), kept, CONST_GIF_WIDTH, CONST_GIF_SPEED, colours, size / 1e6, time.perf_counter() - t0))


def Point_Images(out: str, sheet: tp.Optional[str], note: tp.Callable[[str], None]) -> None:
    """
    Description:
        The cell posed at every point of Constants.yaml (its own scene: Keypoints.Core.Pose_Cell) from the fixed camera, one
        image each into out/Points/, and their contact sheet.

    Args:
        (1) out [string]: The folder (Points/ below it).
        (2) sheet [string]: The contact sheet's file, or None.
        (3) note [function]: Prints a line.
    """

    began = time.perf_counter()
    cell, meta, constants = Load_Cell_Configuration(), Read_PCB_Meta(), Load_Constants()
    simulation = Load_Simulation(constants)
    model = Build_Cell_Model(cell)
    kinematics = Robot_Kinematics_Cls(model)
    width, height = CONST_RESOLUTIONS['1080p']
    renderer = Frame_Renderer_Cls(model, width, height, CONST_CAMERA)
    tiles, labels = [], []
    for name in constants.Points:
        Pose_Cell(kinematics, constants, simulation, cell, meta, name)
        image = renderer.Render(kinematics.Data)
        Save_Image(Label_Frame(image, [name, constants.Points[name].Description]), os.path.join(out, 'Points', name + '.png'))
        tiles.append(np.asarray(Image.fromarray(image).resize(CONST_TILE, Image.Resampling.LANCZOS)))
        labels.append(name)
    renderer.Close()
    note('[INFO] %d point images in %s (%.1f s)' % (len(tiles), _Shown(os.path.join(out, 'Points')),
                                                       time.perf_counter() - began))
    if sheet is not None:
        Save_Image(Contact_Sheet(tiles, labels, CONST_SHEET_COLUMNS), sheet)
        note('[INFO] %s: %d points, %.1f MB' % (_Shown(sheet), len(tiles), os.path.getsize(sheet) / 1e6))


def Collision_Image(path: str, trajectories: tp.Optional[str], note: tp.Callable[[str], None]) -> None:
    """
    Description:
        The collision structure: the check model (the cell with the table top as boxes, Collision.Core) at the taught
        program's first sample, only its collision geom groups shown (1 and 3: the robot's and everything else's), each part in
        its colour (CONST_COLLISION_PARTS, materials off), from the fixed camera, with a legend.

    Args:
        (1) path [string]: The PNG file.
        (2) trajectories [string]: The trajectories' folder, or None.
        (3) note [function]: Prints a line.
    """

    cell = Load_Cell_Configuration()
    model = Build_Planning_Model(cell)
    kinematics = Robot_Kinematics_Cls(model)
    trajectory = Load('taught', trajectories)[3]
    collidable = (model.geom_contype > 0) | (model.geom_conaffinity > 0)
    groups = sorted({int(model.geom_group[g]) for g in np.flatnonzero(collidable)})
    counts = {}
    for g in np.flatnonzero(collidable):
        body = model.body(model.geom_bodyid[g]).name
        for name, rgba, owns in CONST_COLLISION_PARTS:
            if owns(body):
                model.geom_rgba[g], model.geom_matid[g] = rgba, -1
                counts[name] = counts.get(name, 0) + 1
                break
    visual = np.flatnonzero(~collidable)
    model.geom_rgba[visual, 3] = 0.0
    Pose_Sample(kinematics, trajectory, 0)
    mujoco.mj_forward(model, kinematics.Data)
    renderer = Frame_Renderer_Cls(model, *CONST_COLLISION_SIZE, CONST_CAMERA, groups=groups)
    image = Image.fromarray(renderer.Render(kinematics.Data))
    renderer.Close()
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default(size=26)
    x, y = 24, 24
    for name, rgba, _ in CONST_COLLISION_PARTS:
        colour = tuple(int(255 * c) for c in rgba[:3])
        draw.rectangle([x, y + 4, x + 22, y + 26], fill=colour)
        draw.text((x + 34, y), '%s (%d geoms)' % (name, counts.get(name, 0)), font=font, fill=(240, 239, 236))
        y += 38
    draw.text((x, y + 8), 'Collision geoms only (groups %s); visual geoms hidden. Simulation model, not the real cell.'
              % ', '.join(map(str, groups)), font=font, fill=(240, 239, 236))
    Save_Image(np.asarray(image), path)
    note('[INFO] %s: %s' % (_Shown(path), ', '.join('%s %d' % kv for kv in counts.items())))


def main():
    """
    Description:
        The comparison's media (FEATURES.md F14), every one from the same fixed camera (CONST_CAMERA): the physics run of each
        program as an H.264 video (real time, 60 fps, 1920 x 1080; --resolution 1440p adds or picks 2560 x 1440) with the step
        label, and its GIF preview (960 px, 4 x speed, 15 fps, at most 10 MB) into Media/; the cell posed at every point
        (Data/Generated/Report/Points/) and their contact sheet into Media/; the collision structure. The trajectories are
        refused when stale (build them with build_program.py). Exit 0, 2 when a run or a medium cannot be made.

            .venv/bin/python Example/Report/make_media.py [--out DIR] [--media DIR] [--program taught|optimized ...]
                [--resolution 1080p|1440p ...] [--trajectories DIR] [--until S] [--skip-videos] [--skip-points] [--skip-collision]
    """

    parser = argparse.ArgumentParser(description='The comparison\'s media: videos, GIF previews, the key points, the collision structure.')
    parser.add_argument('--out', default=CONST_OUT, help='the full media (default: %s)' % os.path.relpath(CONST_OUT, CONST_ROOT))
    parser.add_argument('--media', default=CONST_MEDIA, help='the README\'s previews and contact sheet (default: Media)')
    parser.add_argument('--program', nargs='+', default=list(CONST_VERSIONS), choices=list(CONST_VERSIONS), help='the programs (default: both)')
    parser.add_argument('--resolution', nargs='+', default=['1080p'], choices=list(CONST_RESOLUTIONS), help='the videos (default: 1080p)')
    parser.add_argument('--trajectories', help='the folder of Program.npz and Program_Optimized.npz (default: Data/Generated/Trajectories)')
    parser.add_argument('--until', type=float, help='end the videos and previews at this time of the cycle (s; default: the whole cycle)')
    parser.add_argument('--skip-videos', action='store_true', help='no videos and previews')
    parser.add_argument('--skip-points', action='store_true', help='no point images and contact sheet')
    parser.add_argument('--skip-collision', action='store_true', help='no collision image')
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('make_media')
    note = lambda line: print(log.Line(line), flush=True)
    began = time.perf_counter()
    run_program = _Script('Simulation/run_program.py')
    try:
        if not args.skip_videos:
            with multiprocessing.get_context('spawn').Pool(len(args.program), initializer=Configure_MuJoCo_Log) as pool:
                runs = pool.map(Record, [(v, args.trajectories) for v in args.program], chunksize=1)
            model = Build_Cell_Model(Load_Cell_Configuration())
            for version, trajectory, record, seconds in runs:
                note('[INFO] %s: physics run recorded in %.1f s' % (version, seconds))
                Videos(model, version, trajectory, record, args.resolution, args.out,
                       os.path.join(args.media, 'Cycle_%s.gif' % CONST_VERSIONS[version][1]), args.until, note)
        if not args.skip_points:
            Point_Images(args.out, os.path.join(args.media, 'Key_Points.png'), note)
        if not args.skip_collision:
            Collision_Image(os.path.join(args.out, 'Collision_Structure.png'), args.trajectories, note)
    except (run_program.Run_Error_Cls, Simulation_Error_Cls, Media_Error_Cls) as error:
        note('[ERROR] %s' % error)
        log.Close()
        return 2
    note('[INFO] Done in %.1f s. Simulation results only: not validated for the real cell.' % (time.perf_counter() - began))
    log.Close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
