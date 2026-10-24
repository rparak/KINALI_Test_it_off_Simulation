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
# File Name: ../Example/Simulation/run_cycle.py
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
# Custom Lib.:
#   ../Configuration/Core - paths, the PCB data, the logs
from Configuration.Core import (Configure_MuJoCo_Log, Read_PCB_Meta, Load_Cell_Configuration, Load_Keypoints, Script_Log_Cls,
                                Configuration_Error_Cls, CONST_REFERENCE_PLAN,
                                CONST_REPORTS, CONST_REPORT_NAME, CONST_ROOT)
#   ../Scene/Core - the cell
from Scene.Core import Build_Cell_Model, Scene_Error_Cls
#   ../Simulation/Core - the run, the report, the acceptance
from Simulation.Core import Read_Legacy_Plan, Run_Cycle, Format_Report, Check_Acceptance, Simulation_Error_Cls

"""
Description:
    Initialization of constants.
"""
# Exit codes: every acceptance criterion met, one failed, the run could not run or became unstable.
#   Unit [int]
CONST_EXIT_PASSED, CONST_EXIT_FAILED, CONST_EXIT_ERROR = 0, 1, 2


def main():
    """
    Description:
        The physics run of a plan (default: the verified reference plan Data/Reference/cell_plan.npz) through the project's own Simulation
        domain: the report of SPEC §4.6 on stdout and in DIR/report.txt, the acceptance of SPEC §10.1 as
        [INFO] / [ERROR] lines on stderr, everything also in logger/run_cycle.log. Exit 0 when every
        criterion is met, 1 when one fails, 2 when the run cannot start or becomes unstable.

            .venv/bin/python Example/Simulation/run_cycle.py [--plan Data/Reference/cell_plan.npz] [--out Data/Generated/Reports/] [--progress]
    """

    parser = argparse.ArgumentParser(description='Physics run of a cell plan: report and acceptance checks.')
    parser.add_argument('--plan', default=CONST_REFERENCE_PLAN, help='the plan (default: %s)' % os.path.relpath(CONST_REFERENCE_PLAN, CONST_ROOT))
    parser.add_argument('--out', default=CONST_REPORTS,
                        help='write %s into this directory (default: %s)' % (CONST_REPORT_NAME, os.path.relpath(CONST_REPORTS, CONST_ROOT)))
    parser.add_argument('--progress', action='store_true', help='print the simulated time and phase every 2 s (stderr)')
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('run_cycle')
    note = lambda line: print(log.Line(line), file=sys.stderr, flush=True)
    progress = (lambda t, phase: note('[INFO] t = %5.1f s  %s' % (t, phase))) if args.progress else None
    try:
        cell, keypoints = Load_Cell_Configuration(), Load_Keypoints()
        for level, message in cell.Notes + keypoints.Notes:
            note('[%s] %s' % (level, message))
        result = Run_Cycle(Build_Cell_Model(cell), Read_Legacy_Plan(os.path.abspath(args.plan)), Read_PCB_Meta(), cell,
                           keypoints.Loader['home'], progress)
    except (Configuration_Error_Cls, Scene_Error_Cls, Simulation_Error_Cls) as error:
        note('[ERROR] %s' % error)
        log.Close()
        return CONST_EXIT_ERROR
    lines = Format_Report(result)
    for line in lines:
        print(log.Line(line), flush=True)
    os.makedirs(args.out, exist_ok=True)
    report = os.path.join(os.path.abspath(args.out), CONST_REPORT_NAME)
    with open(report, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')
    checks = Check_Acceptance(result, cell)
    for check in checks:
        note('[%s] %s %-55s %s' % ('INFO' if check.Passed else 'ERROR', 'PASS' if check.Passed else 'FAIL', check.Name, check.Detail))
    failed = [c for c in checks if not c.Passed]
    note('[%s] Report written to %s; %d of %d acceptance checks failed. Simulation results only: not validated for the real cell.'
         % ('ERROR' if failed else 'INFO', report, len(failed), len(checks)))
    log.Close()
    return CONST_EXIT_FAILED if failed else CONST_EXIT_PASSED


if __name__ == '__main__':
    sys.exit(main())
