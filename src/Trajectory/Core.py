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
# File Name: ../Trajectory/Core.py
## =========================================================================== ##

# System (Default)
import os
# JSON (JSON encoder and decoder)
import json
# Typing (Support for type hints)
import typing as tp
# Dataclasses (Data Classes)
from dataclasses import dataclass, field
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.: the PCB locations the world state names
#   ../Configuration/Program_Parser - the PCB locations
from Configuration.Program_Parser import CONST_PCB_LOCATIONS

"""
Description:
    Initialization of constants.

    The trajectory file (SPEC §9, FEATURES.md F8): a program sampled every 4 ms - the robot joints and speeds, the
    loader and gripper commands, the vacuum, the step and segment of each sample, and the world state (the PCB's
    location and pose, MG06's lid and handle, the jaws' real position) - as <name>.npz with a CSV export of the per-sample
    columns. This module is the only writer and reader of the format; it knows nothing of MuJoCo and prints nothing.
"""
# The sample period of the format (SPEC §9, brief §5.4: the same as the reference plan).
#   Unit [float] (s)
CONST_DT = 0.004
# The format's version, stored in its metadata; a file of another version is refused.
#   Unit [int]
CONST_FORMAT_VERSION = 1
# The per-sample arrays and their second dimension (1 for a vector).
#   Unit [Dict<string, int>]
CONST_SAMPLE_KEYS = {'t': 1, 'robot_q': 6, 'robot_qd': 6, 'loader_y': 1, 'loader_z': 1, 'gripper': 1, 'vacuum': 1, 'step_index': 1,
                     'step_label': 1, 'segment_id': 1, 'pcb_location': 1, 'pcb_pose': 7, 'mg06_lid': 1, 'mg06_handle': 1, 'jaw': 1}
# The per-segment arrays.
#   Unit [List<string>]
CONST_SEGMENT_KEYS = ['segment_step', 'segment_command', 'segment_start', 'segment_end']
# The CSV export's number format: fixed, so the same trajectory gives the same bytes.
#   Unit [string]
CONST_CSV_FORMAT = '%.9f'


class Trajectory_Error_Cls(ValueError):
    """
    Description:
        A trajectory file cannot be read (missing, another format version, a missing or malformed array). The message
        names the file and what to do; a script prints it verbatim as an [ERROR] line.
    """


@dataclass
class Trajectory_Str:
    """
    Description:
        A program's trajectory: per-sample arrays (N samples, CONST_DT apart), per-segment arrays (K segments, one per
        command), the steps' names, the vacuum and gripper events, and the metadata. Built by Program.Core.Build_Program,
        written and read here.
    """

    # The time of each sample.
    #   Unit [Vector<float> 1xN] (s)
    T: np.ndarray = field(default_factory=lambda: np.zeros(0))
    # The robot joints and their speeds, shoulder_pan .. wrist_3.
    #   Unit [Matrix<float> Nx6] (rad), (rad/s)
    Robot_Q: np.ndarray = field(default_factory=lambda: np.zeros((0, 6)))
    Robot_QD: np.ndarray = field(default_factory=lambda: np.zeros((0, 6)))
    # The loader axis commands and the gripper's jaw command (per jaw).
    #   Unit [Vector<float> 1xN] (m)
    Loader_Y: np.ndarray = field(default_factory=lambda: np.zeros(0))
    Loader_Z: np.ndarray = field(default_factory=lambda: np.zeros(0))
    Gripper: np.ndarray = field(default_factory=lambda: np.zeros(0))
    # The vacuum: 1 on, 0 off.
    #   Unit [Vector<int> 1xN] (-)
    Vacuum: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int8))
    # The step (an index into Step_Names) and the segment (an index into the Segment_* arrays) of each sample.
    #   Unit [Vector<int> 1xN] (-)
    Step_Index: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int32))
    Segment_Id: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int32))
    # The world state: the PCB's location (an index into CONST_PCB_LOCATIONS) and pose (position, quaternion w first),
    # MG06's lid and handle, the jaws' real position (grip while the PCB is between them, the command otherwise).
    #   Unit [Vector<int> 1xN], [Matrix<float> Nx7] (m, -), [Vector<float> 1xN] (rad), (rad), (m)
    PCB_Location: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int8))
    PCB_Pose: np.ndarray = field(default_factory=lambda: np.zeros((0, 7)))
    MG06_Lid: np.ndarray = field(default_factory=lambda: np.zeros(0))
    MG06_Handle: np.ndarray = field(default_factory=lambda: np.zeros(0))
    Jaw: np.ndarray = field(default_factory=lambda: np.zeros(0))
    # The program's step names, in order.
    #   Unit [List<string>]
    Step_Names: tp.List[str] = field(default_factory=list)
    # Per segment (one command): its step (an index), its text, its first and one-past-last sample.
    #   Unit [Vector<int> 1xK], [List<string>], [Vector<int> 1xK], [Vector<int> 1xK]
    Segment_Step: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int32))
    Segment_Command: tp.List[str] = field(default_factory=list)
    Segment_Start: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int32))
    Segment_End: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int32))
    # The events: (time, 'vacuum_on' / 'vacuum_off' / 'gripper_<state>').
    #   Unit [List<Tuple<float, string>>] (s, -)
    Events: tp.List[tp.Tuple[float, str]] = field(default_factory=list)
    # The metadata: the format version, the program, the input files' sha256, the git commit, the units, the joint order,
    # the TCP (SPEC §17).
    #   Unit [Dict<string, Any>]
    Meta: tp.Dict[str, tp.Any] = field(default_factory=dict)


def Write_Trajectory(trajectory: Trajectory_Str, path: str) -> tp.Tuple[str, str]:
    """
    Description:
        Write a trajectory as path (.npz) and the per-sample columns as the CSV beside it (the same name, .csv), creating
        the folder. The same trajectory gives byte-identical files (fixed key order, fixed CSV number format, no
        timestamp in the archive).

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2) path [string]: The .npz file.

    Returns:
        (1) parameter [string]: The .npz file written.
        (2) parameter [string]: The .csv file written.
    """

    path = os.path.abspath(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tr = trajectory
    labels = np.array([tr.Step_Names[i] for i in tr.Step_Index]) if len(tr.Step_Index) else np.zeros(0, '<U1')
    arrays = {'t': tr.T, 'robot_q': tr.Robot_Q, 'robot_qd': tr.Robot_QD, 'loader_y': tr.Loader_Y, 'loader_z': tr.Loader_Z,
              'gripper': tr.Gripper, 'vacuum': tr.Vacuum, 'step_index': tr.Step_Index, 'step_label': labels, 'segment_id': tr.Segment_Id,
              'pcb_location': tr.PCB_Location, 'pcb_pose': tr.PCB_Pose, 'mg06_lid': tr.MG06_Lid, 'mg06_handle': tr.MG06_Handle, 'jaw': tr.Jaw,
              'step_names': np.array(tr.Step_Names), 'segment_step': tr.Segment_Step, 'segment_command': np.array(tr.Segment_Command),
              'segment_start': tr.Segment_Start, 'segment_end': tr.Segment_End,
              'events': np.array(json.dumps([[float(t), str(n)] for t, n in tr.Events])),
              'meta': np.array(json.dumps(dict(tr.Meta, format_version=CONST_FORMAT_VERSION, dt=CONST_DT), sort_keys=True))}
    with open(path, 'wb') as handle:
        np.savez(handle, **arrays)
    csv_path = os.path.splitext(path)[0] + '.csv'
    header = (['t'] + ['q%d' % j for j in range(1, 7)] + ['qd%d' % j for j in range(1, 7)]
              + ['loader_y', 'loader_z', 'gripper', 'vacuum', 'step_index', 'step_label', 'segment_id', 'pcb_location', 'mg06_lid',
                 'mg06_handle', 'jaw'])
    number = CONST_CSV_FORMAT
    with open(csv_path, 'w', encoding='utf-8', newline='\n') as handle:
        handle.write(','.join(header) + '\n')
        for i in range(len(tr.T)):
            values = [number % tr.T[i]] + [number % v for v in tr.Robot_Q[i]] + [number % v for v in tr.Robot_QD[i]]
            values += [number % tr.Loader_Y[i], number % tr.Loader_Z[i], number % tr.Gripper[i], '%d' % tr.Vacuum[i], '%d' % tr.Step_Index[i],
                       labels[i], '%d' % tr.Segment_Id[i], CONST_PCB_LOCATIONS[tr.PCB_Location[i]], number % tr.MG06_Lid[i],
                       number % tr.MG06_Handle[i], number % tr.Jaw[i]]
            handle.write(','.join(values) + '\n')
    return path, csv_path


def Read_Trajectory(path: str) -> Trajectory_Str:
    """
    Description:
        Read a trajectory .npz written by Write_Trajectory, the only reader of the format: every array present with its
        shape, the format version the current one.

    Args:
        (1) path [string]: The .npz file.

    Returns:
        (1) parameter [Trajectory_Str(object)]: The trajectory.
    """

    try:
        archive = np.load(path)
        data = {k: archive[k] for k in archive.files}
    except (OSError, ValueError) as error:
        raise Trajectory_Error_Cls('The trajectory %s cannot be read (%s). Build it with Example/Program/build_program.py.'
                                   % (path, error)) from error
    missing = [k for k in list(CONST_SAMPLE_KEYS) + CONST_SEGMENT_KEYS + ['step_names', 'events', 'meta'] if k not in data]
    if missing:
        raise Trajectory_Error_Cls('The trajectory %s lacks %s: it is not a trajectory of this format. Rebuild it with '
                                   'Example/Program/build_program.py.' % (path, ', '.join(missing)))
    meta = json.loads(str(data['meta']))
    if meta.get('format_version') != CONST_FORMAT_VERSION:
        raise Trajectory_Error_Cls('The trajectory %s has format version %r, this code reads %d. Rebuild it with Example/Program/build_program.py.'
                                   % (path, meta.get('format_version'), CONST_FORMAT_VERSION))
    n = len(data['t'])
    bad = [k for k, width in CONST_SAMPLE_KEYS.items() if data[k].shape != ((n,) if width == 1 else (n, width))]
    if bad:
        raise Trajectory_Error_Cls('The trajectory %s has arrays of the wrong shape (%s). Rebuild it.' % (path, ', '.join(bad)))
    return Trajectory_Str(T=data['t'], Robot_Q=data['robot_q'], Robot_QD=data['robot_qd'], Loader_Y=data['loader_y'], Loader_Z=data['loader_z'],
                          Gripper=data['gripper'], Vacuum=data['vacuum'], Step_Index=data['step_index'], Segment_Id=data['segment_id'],
                          PCB_Location=data['pcb_location'], PCB_Pose=data['pcb_pose'], MG06_Lid=data['mg06_lid'], MG06_Handle=data['mg06_handle'],
                          Jaw=data['jaw'], Step_Names=[str(s) for s in data['step_names']], Segment_Step=data['segment_step'],
                          Segment_Command=[str(s) for s in data['segment_command']], Segment_Start=data['segment_start'],
                          Segment_End=data['segment_end'], Events=[(float(t), str(e)) for t, e in json.loads(str(data['events']))], Meta=meta)
