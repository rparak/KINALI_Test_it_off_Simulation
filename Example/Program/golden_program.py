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
# File Name: ../Example/Program/golden_program.py
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
# JSON (JSON encoder and decoder)
import json
# Hashlib (Secure hashes)
import hashlib
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.:
#   ../Configuration/Core, ../Configuration/Program_Parser - the cell, the program
from Configuration.Core import Configure_MuJoCo_Log, Load_Cell_Configuration, Read_PCB_Meta, Script_Log_Cls
from Configuration.Program_Parser import Load_Constants, Load_Simulation, Load_Program
#   ../Collision/Core, ../Kinematics/Core - the planning model
from Collision.Core import Build_Planning_Model, Robot_Collision_Cls
from Kinematics.Core import Robot_Kinematics_Cls
#   ../Program/Core, ../Keypoints/Core - the build, the program check, the point table
from Program.Core import Build_Program, Check_Trajectory, Format_Check_Report, Executed_Commands
from Keypoints.Core import Check_Keypoints, Format_Check_Table

"""
Description:
    Initialization of constants.

    The golden data of the taught program (test/Program/data/): golden_program.json (each numeric trajectory array's sha256,
    dtype and shape; the cycle time, the sample count, the events with their times as float hex, the steps, the segments'
    texts, every executed command's values as float hex), golden_check_report.txt (the program check's report) and
    golden_keypoint_table.txt (the point table). A change to the program or its build re-captures them in its own commit,
    listing what changed (SPEC §10.2). This script is the one reader and writer of the format; test/Program/test_golden.py
    imports it.
"""
# The golden data folder and its files.
#   Unit [string], [List<string>]
CONST_GOLDEN = os.path.join(CONST_REPO_ROOT, 'test', 'Program', 'data')
CONST_FILES = ['golden_program.json', 'golden_check_report.txt', 'golden_keypoint_table.txt']


def Arrays(trajectory: tp.Any) -> tp.Dict[str, np.ndarray]:
    """
    Description:
        The trajectory's numeric arrays by their file names.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.

    Returns:
        (1) parameter [Dict<string, np.ndarray>]: The arrays.
    """

    tr = trajectory
    return {'t': tr.T, 'robot_q': tr.Robot_Q, 'robot_qd': tr.Robot_QD, 'loader_y': tr.Loader_Y, 'loader_z': tr.Loader_Z, 'gripper': tr.Gripper,
            'vacuum': tr.Vacuum, 'pcb_location': tr.PCB_Location, 'pcb_pose': tr.PCB_Pose, 'mg06_lid': tr.MG06_Lid, 'mg06_handle': tr.MG06_Handle,
            'jaw': tr.Jaw, 'step_index': tr.Step_Index, 'segment_id': tr.Segment_Id}


def Capture(source: str = '') -> tp.Dict[str, tp.Any]:
    """
    Description:
        The golden data of the shipped taught program, built now: the program record (a dict) and the two reports' lines.

    Args:
        (1) source [string]: The capture's provenance, stored in the record (not compared).

    Returns:
        (1) parameter [Dict<string, Any>]: 'program' (dict), 'report' and 'table' (List<string>).
    """

    cell, meta = Load_Cell_Configuration(), Read_PCB_Meta()
    constants = Load_Constants()
    simulation = Load_Simulation(constants)
    program = Load_Program(constants, simulation, cell)
    model = Build_Planning_Model(cell)
    collision = Robot_Collision_Cls(model)
    kinematics = Robot_Kinematics_Cls(model, collision.Data)
    tr, notes = Build_Program(program, cell, meta, kinematics, {})
    arrays = {name: {'sha256': hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest(), 'dtype': str(value.dtype),
                     'shape': list(value.shape)} for name, value in Arrays(tr).items()}
    commands = []
    for _, step, c, text in Executed_Commands(program):
        commands.append({'step': step, 'kind': c.Kind, 'text': text, 'target': c.Target, 'joints_rad_hex': [float(q).hex() for q in c.Joints],
                         'speed': float(c.Speed).hex(), 'duration': float(c.Duration).hex(), 'timeout': float(c.Timeout).hex(), 'axis': c.Axis,
                         'position': float(c.Position).hex(), 'jaw': float(c.Jaw).hex()})
    record = {'source': source, 'cycle_time': float(tr.T[-1]), 'samples': len(tr.T), 'arrays': arrays,
              'events': [[float(t).hex(), e] for t, e in tr.Events], 'step_names': list(tr.Step_Names),
              'segment_commands': list(tr.Segment_Command), 'commands': commands}
    report = Format_Check_Report(Check_Trajectory(tr, program, notes, kinematics, collision, cell, meta), tr, cell)
    table = Format_Check_Table(Check_Keypoints(kinematics, collision, constants, simulation, cell, meta))
    return {'program': record, 'report': report, 'table': table}


def Read_Golden(folder: str = CONST_GOLDEN) -> tp.Dict[str, tp.Any]:
    """
    Description:
        The golden data written by Write_Golden.

    Args:
        (1) folder [string]: The folder.

    Returns:
        (1) parameter [Dict<string, Any>]: As Capture.
    """

    with open(os.path.join(folder, CONST_FILES[0]), encoding='utf-8') as handle:
        record = json.load(handle)
    lines = []
    for name in CONST_FILES[1:]:
        with open(os.path.join(folder, name), encoding='utf-8') as handle:
            lines.append(handle.read().splitlines())
    return {'program': record, 'report': lines[0], 'table': lines[1]}


def Write_Golden(golden: tp.Dict[str, tp.Any], folder: str) -> None:
    """
    Description:
        Write the golden data into a folder (CONST_FILES).

    Args:
        (1) golden [Dict<string, Any>]: As Capture.
        (2) folder [string]: The folder.
    """

    os.makedirs(folder, exist_ok=True)
    with open(os.path.join(folder, CONST_FILES[0]), 'w', encoding='utf-8') as handle:
        handle.write(json.dumps(golden['program'], indent=1) + '\n')
    for name, key in zip(CONST_FILES[1:], ('report', 'table')):
        with open(os.path.join(folder, name), 'w', encoding='utf-8') as handle:
            handle.write('\n'.join(golden[key]) + '\n')


def Compare_Golden(now: tp.Dict[str, tp.Any], golden: tp.Dict[str, tp.Any]) -> tp.List[str]:
    """
    Description:
        The differences between two captures, one line each: each array whose bytes differ, the cycle time, the sample count,
        the events, the steps, the segments, each command field, each report or table line ('source' is not compared).

    Args:
        (1 - 2) now, golden [Dict<string, Any>]: As Capture.

    Returns:
        (1) parameter [List<string>]: The differences, empty when identical.
    """

    a, b = now['program'], golden['program']
    out = ['array %s: %s -> %s' % (k, b['arrays'].get(k), a['arrays'].get(k)) for k in sorted(set(a['arrays']) | set(b['arrays']))
           if a['arrays'].get(k) != b['arrays'].get(k)]
    for key in ('cycle_time', 'samples', 'events', 'step_names', 'segment_commands'):
        if a[key] != b[key]:
            out.append('%s: %s -> %s' % (key, b[key] if not isinstance(b[key], list) else '%d entries' % len(b[key]),
                                         a[key] if not isinstance(a[key], list) else '%d entries' % len(a[key])))
    if len(a['commands']) != len(b['commands']):
        out.append('commands: %d -> %d' % (len(b['commands']), len(a['commands'])))
    for i, (x, y) in enumerate(zip(a['commands'], b['commands'])):
        out += ['command %d (%s) %s: %s -> %s' % (i, x['text'], k, y.get(k), x.get(k)) for k in sorted(set(x) | set(y)) if x.get(k) != y.get(k)]
    for key in ('report', 'table'):
        if now[key] != golden[key]:
            first = next((i for i, (x, y) in enumerate(zip(now[key], golden[key])) if x != y), min(len(now[key]), len(golden[key])))
            out.append('%s line %d: %r -> %r' % (key, first + 1, golden[key][first] if first < len(golden[key]) else None,
                                                  now[key][first] if first < len(now[key]) else None))
    return out


def main():
    """
    Description:
        Build the taught program and compare its golden data with test/Program/data/ (default) or another folder, listing every
        difference (exit 1 if there is one), or write it into a folder with --out (a re-capture: the source names it).

            .venv/bin/python Example/Program/golden_program.py [--compare DIR | --out DIR --source TEXT]
    """

    parser = argparse.ArgumentParser(description='The taught program\'s golden data: compare or write.')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--compare', default=CONST_GOLDEN, help='compare with this folder (default: test/Program/data/)')
    group.add_argument('--out', default=None, help='write the golden data into this folder instead of comparing')
    parser.add_argument('--source', default='', help='the capture\'s provenance, stored with --out')
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('golden_program')
    now = Capture(args.source)
    if args.out is not None:
        Write_Golden(now, args.out)
        print(log.Line('[INFO] Wrote %s into %s: cycle %.3f s, %d samples, %d commands.' % (', '.join(CONST_FILES), args.out,
                                                                                         now['program']['cycle_time'], now['program']['samples'],
                                                                                         len(now['program']['commands']))))
        log.Close()
        return 0
    differences = Compare_Golden(now, Read_Golden(args.compare))
    print(log.Line('[%s] %d difference(s) against %s' % ('INFO' if not differences else 'WARNING', len(differences), args.compare)))
    for line in differences:
        print(log.Line('  ' + line))
    log.Close()
    return 1 if differences else 0


if __name__ == '__main__':
    sys.exit(main())
