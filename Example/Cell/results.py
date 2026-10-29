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
# File Name: ../Example/Cell/results.py
## =========================================================================== ##

# OS (Operating system interfaces)
import os
# Regular expressions (Regular expression operations)
import re
# Typing (Support for type hints)
import typing as tp

"""
Description:
    Initialization of constants.

    The comparison of the two programs that Example/cell.py run prints (owner, 2026-10-29: numbers only): per program its
    cycle time, its throughput (3600 s / the cycle time) and its acceptance (the checks of SPEC §10.1 met), and the Optimized
    program's change against the Default program in %. The one formatter of these numbers, and the one writer of the README's
    results block (between CONST_BLOCK_START and CONST_BLOCK_END; owner, 2026-10-29: cell.py all --recapture regenerates it
    from the physics baselines, nothing else in the README is written by code; test/Docs/test_readme.py compares it).
"""
# The programs in their order, by their run names, with the names shown (readme-writer project reference > Statuses).
#   Unit [Dict<string, string>]
CONST_NAMES = {'taught': 'Default program', 'optimized': 'Optimized program'}
# Seconds per hour: the throughput is CONST_HOUR / the cycle time.
#   Unit [float] (s)
CONST_HOUR = 3600.0
# The README's results block markers, and a physics baseline's report by run name (test/Simulation/data/).
#   Unit [string], [string] (a file name pattern)
CONST_BLOCK_START, CONST_BLOCK_END = '<!-- results:start -->', '<!-- results:end -->'
CONST_BASELINE_REPORT = 'baseline_report_%s.txt'


def Change(default: float, optimized: float) -> float:
    """
    Description:
        The Optimized program's value against the Default program's, as a change in %.

    Args:
        (1 - 2) default, optimized [float]: The two values.

    Returns:
        (1) parameter [float]: 100 (optimized / default - 1).
    """

    return 100.0 * (optimized / default - 1.0)


def Table_Lines(cycles: tp.Dict[str, float], passed: tp.Dict[str, tp.Tuple[int, int]]) -> tp.List[str]:
    """
    Description:
        The comparison table printed by cell.py run: a row per program (cycle time [s] to 1 ms, throughput [PCB/h] to 0.1,
        acceptance as met / all), then the change row (the cycle time's and the throughput's change in %, to 0.1 %).

    Args:
        (1) cycles [Dict<string, float>]: The cycle time per program ('taught', 'optimized') (s).
        (2) passed [Dict<string, Tuple<int, int>>]: The acceptance checks met and in all, per program.

    Returns:
        (1) parameter [List<string>]: The lines.
    """

    lines = ['%-20s %10s %19s  %s' % ('program', 'cycle [s]', 'throughput [PCB/h]', 'acceptance')]
    for key, name in CONST_NAMES.items():
        met, total = passed[key]
        lines.append('%-20s %10.3f %19.1f  %d/%d %s' % (name, cycles[key], CONST_HOUR / cycles[key], met, total, 'pass' if met == total else 'FAIL'))
    lines.append('%-20s %+8.1f %% %+17.1f %%' % ('change', Change(cycles['taught'], cycles['optimized']),
                                                 Change(CONST_HOUR / cycles['taught'], CONST_HOUR / cycles['optimized'])))
    return lines


def Baseline_Results(folder: str) -> tp.Tuple[tp.Dict[str, float], tp.Dict[str, tp.Tuple[int, int]]]:
    """
    Description:
        Each program's cycle time and acceptance from its physics baseline's report: its first line (cycle N s) and its last
        line (the checks met, (M of T)).

    Args:
        (1) folder [string]: The baseline folder (test/Simulation/data/).

    Returns:
        (1) parameter [Dict<string, float>]: The cycle time per program (s).
        (2) parameter [Dict<string, Tuple<int, int>>]: The checks met and in all, per program.
    """

    cycles, passed = {}, {}
    for key in CONST_NAMES:
        with open(os.path.join(folder, CONST_BASELINE_REPORT % key), encoding='utf-8') as handle:
            lines = handle.read().splitlines()
        cycles[key] = float(re.fullmatch(r'program \S+: physics run of its trajectory, cycle (\d+\.\d{3}) s', lines[0]).group(1))
        met, total = re.search(r'\((\d+) of (\d+)\)', lines[-1]).groups()
        passed[key] = (int(met), int(total))
    return cycles, passed


def _Percent(value: float) -> str:
    """
    Description:
        A change in % as the README writes it: one decimal, a sign, the minus as U+2212.

    Args:
        (1) value [float]: The change (%).

    Returns:
        (1) parameter [string]: E.g. '−16.0 %', '+19.0 %'.
    """

    return ('%+.1f %%' % value).replace('-', '\u2212')


def Readme_Block(cycles: tp.Dict[str, float], passed: tp.Dict[str, tp.Tuple[int, int]]) -> str:
    """
    Description:
        The README's results block: the markers around a table of the cycle time [s] (to 1 ms), the throughput [PCB/h] (to
        0.1) and the acceptance checks of both programs, with the Optimized program's change ([P]: the physics run).

    Args:
        (1) cycles [Dict<string, float>]: The cycle time per program (s).
        (2) passed [Dict<string, Tuple<int, int>>]: The checks met and in all, per program.

    Returns:
        (1) parameter [string]: The block, markers included, without a final newline.
    """

    default, optimized = cycles['taught'], cycles['optimized']
    accept = ['%d/%d %s' % (met, total, 'pass' if met == total else 'FAIL') for met, total in (passed['taught'], passed['optimized'])]
    return '\n'.join([CONST_BLOCK_START,
                      '| Metric | %s | %s | Change |' % (CONST_NAMES['taught'], CONST_NAMES['optimized']),
                      '|---|---:|---:|---:|',
                      '| Cycle time [s] [P] | %.3f | %.3f | %s |' % (default, optimized, _Percent(Change(default, optimized))),
                      '| Throughput [PCB/h] [P] | %.1f | %.1f | %s |' % (CONST_HOUR / default, CONST_HOUR / optimized,
                                                                         _Percent(Change(CONST_HOUR / default, CONST_HOUR / optimized))),
                      '| Acceptance checks [P] | %s | %s | %s |' % (accept[0], accept[1], 'equal' if accept[0] == accept[1] else '\u2014'),
                      CONST_BLOCK_END])


def Update_Readme(path: str, block: str) -> bool:
    """
    Description:
        Replace the README's results block (from CONST_BLOCK_START to CONST_BLOCK_END) with block; nothing else of the file
        changes. A README without exactly one pair of markers is refused.

    Args:
        (1) path [string]: The README.
        (2) block [string]: The new block (Readme_Block).

    Returns:
        (1) parameter [bool]: Whether the file changed.
    """

    with open(path, encoding='utf-8') as handle:
        text = handle.read()
    if text.count(CONST_BLOCK_START) != 1 or text.count(CONST_BLOCK_END) != 1 or text.index(CONST_BLOCK_START) > text.index(CONST_BLOCK_END):
        raise ValueError('%s must hold one results block (%s ... %s)' % (path, CONST_BLOCK_START, CONST_BLOCK_END))
    start, end = text.index(CONST_BLOCK_START), text.index(CONST_BLOCK_END) + len(CONST_BLOCK_END)
    new = text[:start] + block + text[end:]
    if new == text:
        return False
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(new)
    return True
