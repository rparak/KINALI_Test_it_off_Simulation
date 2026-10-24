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
# File Name: ../Example/Program/build_program.py
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
# Platform (Access to the platform's identifying data)
import platform
# Subprocess (Subprocess management)
import subprocess
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Custom Lib.:
#   ../Configuration/Core - the configuration, the program, the logs, the folders
from Configuration.Core import (Configure_MuJoCo_Log, Load_Cell_Configuration, Read_PCB_Meta, Script_Log_Cls, Configuration_Error_Cls,
                                CONST_CELL_YAML, CONST_FORK_PATH, CONST_TRAJECTORIES, CONST_REPORTS, CONST_REFERENCE_PLAN)
#   ../Configuration/Program_Parser - the program's three files
from Configuration.Program_Parser import (Load_Constants, Load_Simulation, Load_Program, CONST_CONSTANTS_YAML, CONST_SIMULATION_YAML,
                                          CONST_PROGRAM_YAML, CONST_OPTIMIZATION_YAML)
#   ../Scene/Core - the cell's errors
from Scene.Core import Scene_Error_Cls
#   ../Kinematics/Core, ../Collision/Core - FK, IK and exact distances on the planning model (the table included)
from Kinematics.Core import Robot_Kinematics_Cls
from Collision.Core import Build_Planning_Model, Robot_Collision_Cls
#   ../Program/Core - the build and the check
from Program.Core import Build_Program, Check_Trajectory, Format_Check_Report, File_Hash, Program_Error_Cls
#   ../Trajectory/Core - the trajectory file
from Trajectory.Core import Trajectory_Str, Write_Trajectory, CONST_DT
#   ../Optimizer/Core - an optimized program's inputs and schedule
from Optimizer.Core import Stale_Inputs, Schedule_Differences
#   ../Simulation/Core - the reference plan (the legacy diagnostic)
from Simulation.Core import Read_Legacy_Plan, Simulation_Error_Cls

"""
Description:
    Initialization of constants.
"""
# The legacy diagnostic's blocks (--compare-legacy): each a label, the program's first step of the block and the reference
# plan's first phase of it; a block ends where the next begins. The cut points are where both run the same process step
# (FEATURES.md F8).
#   Unit [List<Tuple<string>>]
CONST_LEGACY_BLOCKS = [('pick from the input crate', 'S1_Approach_Input', 'robot: pick the PCB from crate 1 (vacuum)'),
                       ('to the loader, handover, home', 'S3_To_Loader', 'robot: put the PCB into the loader gripper'),
                       ('loader: insert and park', 'S6_Loader_Insert', 'loader: up, towards MG06'),
                       ('to MG06, close it', 'S8_To_MG06', 'robot: to the MG06 handle'),
                       ('leave, test (wait), return', 'S10_Leave_MG06', 'robot: leave MG06 (fixture closed)'),
                       ('open MG06, home', 'S13_Open_MG06', 'robot: open MG06'),
                       ('loader: fetch and return', 'S15_Loader_Fetch', 'loader: back to MG06'),
                       ('take, place, home', 'S17_To_Take', 'robot: take the PCB from the loader (vacuum)')]


def Git_State() -> tp.Tuple[str, bool]:
    """
    Description:
        The repository's commit and whether its working tree has changes (for the trajectory's metadata).

    Returns:
        (1) parameter [string]: The commit hash, or 'unknown' outside git.
        (2) parameter [bool]: True when the working tree differs from the commit.
    """

    try:
        commit = subprocess.run(['git', '-C', CONST_REPO_ROOT, 'rev-parse', 'HEAD'], capture_output=True, text=True, check=True).stdout.strip()
        status = subprocess.run(['git', '-C', CONST_REPO_ROOT, 'status', '--porcelain'], capture_output=True, text=True, check=True)
        dirty = bool(status.stdout.strip())
        return commit, dirty
    except (OSError, subprocess.CalledProcessError):
        return 'unknown', True


def Build_Meta(program_path: str, optimized: bool = False) -> tp.Dict[str, tp.Any]:
    """
    Description:
        The trajectory's metadata: the sha256 of every input file (an optimized program's also Configuration/
        Optimization.yaml's), the git commit, the platform and the versions (B5: byte-identical outputs are promised on one
        machine and platform only).

    Args:
        (1) program_path [string]: The program file.
        (2) optimized [bool]: Whether it is an optimized program.

    Returns:
        (1) parameter [Dict<string, Any>]: The metadata.
    """

    commit, dirty = Git_State()
    inputs = {name: File_Hash(path) for name, path in (('program', program_path), ('constants', CONST_CONSTANTS_YAML),
                                                        ('simulation', CONST_SIMULATION_YAML), ('cell', CONST_CELL_YAML), ('fork_path',
                                                                                                                           CONST_FORK_PATH))}
    if optimized:
        inputs['optimization'] = File_Hash(CONST_OPTIMIZATION_YAML)
    return {'inputs_sha256': inputs, 'git_commit': commit, 'git_dirty': dirty,
            'platform': {'system': platform.system(), 'machine': platform.machine(), 'python': platform.python_version(),
                         'numpy': np.__version__, 'mujoco': mujoco.__version__}}


def Compare_Legacy(trajectory: Trajectory_Str, points_joints: tp.Dict[str, np.ndarray]) -> tp.List[str]:
    """
    Description:
        The diagnostic against the reference plan Data/Reference/cell_plan.npz (never an acceptance bound): the cycle times;
        per block (CONST_LEGACY_BLOCKS) both starts and durations and, where the durations agree to a sample, the largest
        joint difference with the starts aligned; the vacuum events; the largest difference between a keypoint's stored
        joints and the plan's nearest row.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The program's trajectory.
        (2) points_joints [Dict<string, Vector<float> 1x6>]: The joints of the points the program uses (rad).

    Returns:
        (1) parameter [List<string>]: The lines.
    """

    plan = Read_Legacy_Plan(CONST_REFERENCE_PLAN)
    starts_program = [float(trajectory.T[np.flatnonzero(trajectory.Step_Index == trajectory.Step_Names.index(step))[0]])
                      for _, step, _ in CONST_LEGACY_BLOCKS]
    phases = {label: t for t, label in reversed(plan.Phases)}
    starts_legacy = [float(phases[label]) for _, _, label in CONST_LEGACY_BLOCKS]
    # Both cycle times count the samples after the start (the plan's T is its row count x 4 ms; the program's last sample).
    end_program, end_legacy = float(trajectory.T[-1]), plan.T_End
    lines = ['legacy diagnostic (Data/Reference/cell_plan.npz; a comparison, not an acceptance bound):',
             '  cycle time: program %.3f s, legacy %.3f s, difference %+.3f s' % (end_program, end_legacy, end_program - end_legacy),
             '  %-30s %9s %9s %9s %9s %9s %s' % ('block', 'start p', 'start l', 'dur p', 'dur l', 'dur p-l', 'max |dq| aligned')]
    for k, (label, _, _) in enumerate(CONST_LEGACY_BLOCKS):
        p0, l0 = starts_program[k], starts_legacy[k]
        p1 = starts_program[k + 1] if k + 1 < len(CONST_LEGACY_BLOCKS) else end_program
        l1 = starts_legacy[k + 1] if k + 1 < len(CONST_LEGACY_BLOCKS) else end_legacy
        aligned = '-'
        if abs((p1 - p0) - (l1 - l0)) < CONST_DT / 2:
            n = int(round((p1 - p0) / CONST_DT))
            i, j = int(round(p0 / CONST_DT)), int(round(l0 / CONST_DT))
            # The plan's row k is the state after k + 1 samples (its first row is the first sample after the start).
            dq = np.max(np.abs(trajectory.Robot_Q[i + 1:i + n] - plan.Q[j:j + n - 1]), axis=1)
            worst = int(np.argmax(dq))
            aligned = '%.2e rad at %.3f s (%s)' % (float(dq[worst]), trajectory.T[i + 1 + worst],
                                                   trajectory.Segment_Command[trajectory.Segment_Id[i + 1 + worst]])
        lines.append('  %-30s %9.3f %9.3f %9.3f %9.3f %+9.3f %s' % (label, p0, l0, p1 - p0, l1 - l0, (p1 - p0) - (l1 - l0), aligned))
    vacuum_p = [(t, e) for t, e in trajectory.Events if e.startswith('vacuum')]
    vacuum_l = [(float(t), e) for t, e in plan.Events]
    for (tp_, ep), (tl, el) in zip(vacuum_p, vacuum_l):
        lines.append('  %-12s program %8.3f s, legacy %8.3f s, %+.3f s' % (ep, tp_, tl, tp_ - tl))
    nearest = {name: float(np.min(np.max(np.abs(plan.Q - q), axis=1))) for name, q in points_joints.items()}
    in_plan = {n: d for n, d in nearest.items() if d < 1e-6}
    lines.append('  points the program stops at: %d; %d are rows of the plan (their joints within %.1e rad of the nearest row), not in it: %s'
                 % (len(nearest), len(in_plan), max(in_plan.values()), ', '.join(sorted(set(nearest) - set(in_plan))) or 'none'))
    return lines


def main():
    """
    Description:
        Build a program into its trajectory (Data/Generated/Trajectories/<program>.npz and .csv) and check it (the report:
        stdout and Data/Generated/Reports/<program>_check.txt); exit 0 when the check passes, 1 when it fails or the
        program cannot be built. --compare-legacy adds the diagnostic against the reference plan.

            .venv/bin/python Example/Program/build_program.py [--program FILE] [--out DIR] [--compare-legacy]
    """

    parser = argparse.ArgumentParser(description='Build and check a program (default: Configuration/Program.yaml).')
    parser.add_argument('--program', default=CONST_PROGRAM_YAML, help='the program file (with Configuration/Constants.yaml and Simulation.yaml)')
    parser.add_argument('--out', help='the folder for the trajectory and the report (default: Data/Generated/Trajectories and Reports)')
    parser.add_argument('--compare-legacy', action='store_true', help='add the diagnostic against Data/Reference/cell_plan.npz')
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('build_program')
    try:
        cell, meta = Load_Cell_Configuration(), Read_PCB_Meta()
        constants = Load_Constants()
        program = Load_Program(constants, Load_Simulation(constants), cell, args.program)
        for level, message in cell.Notes + program.Notes:
            print(log.Line('[%s] %s' % (level, message)))
        model = Build_Planning_Model(cell)
        collision = Robot_Collision_Cls(model)
        kinematics = Robot_Kinematics_Cls(model, collision.Data)
        stale = Stale_Inputs(program, model)
        if stale:
            print(log.Line('[ERROR] %s was optimized from other inputs (%s changed since); make it again: .venv/bin/python '
                           'Example/Program/optimize_program.py' % (args.program, ', '.join(stale))))
            log.Close()
            return 1
        trajectory, notes = Build_Program(program, cell, meta, kinematics, Build_Meta(args.program, program.Optimized is not None))
        differences = Schedule_Differences(program, trajectory)
        if differences:
            print(log.Line('[ERROR] The build of %s does not match its recorded Schedule (another platform, or the file was edited): %s; make '
                           'it again: .venv/bin/python Example/Program/optimize_program.py' % (args.program, '; '.join(differences))))
            log.Close()
            return 1
    except (Configuration_Error_Cls, Scene_Error_Cls, Program_Error_Cls) as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 1
    npz, csv = Write_Trajectory(trajectory, os.path.join(args.out or CONST_TRAJECTORIES, program.Name + '.npz'))
    result = Check_Trajectory(trajectory, program, notes, kinematics, collision, cell, meta)
    lines = Format_Check_Report(result, trajectory, cell)
    if args.compare_legacy:
        try:
            joints = {name: np.array(program.Constants.Points[name].Joints) for name in (k.Name for k in result.Keypoints)}
            lines += Compare_Legacy(trajectory, joints)
        except Simulation_Error_Cls as error:
            lines.append('[WARNING] %s' % error)
    report = os.path.join(args.out or CONST_REPORTS, program.Name + '_check.txt')
    os.makedirs(os.path.dirname(report), exist_ok=True)
    with open(report, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')
    for line in lines:
        print(log.Line(line))
    print(log.Line('[INFO] Wrote %s, %s and %s.' % (npz, csv, report)))
    print(log.Line('[%s] %s: the check %s.' % ('ERROR' if result.Verdict == 'FAIL' else 'INFO', program.Name,
                                               'failed' if result.Verdict == 'FAIL' else 'passed')))
    log.Close()
    return 1 if result.Verdict == 'FAIL' else 0


if __name__ == '__main__':
    sys.exit(main())
