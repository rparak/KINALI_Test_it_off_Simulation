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

# Typing (Support for type hints)
import typing as tp

"""
Description:
    Initialization of constants.

    The comparison of the two programs that Example/cell.py run prints (owner, 2026-10-29: numbers only): per program its
    cycle time, its throughput (3600 s / the cycle time) and its acceptance (the checks of SPEC §10.1 met), and the Optimized
    program's change against the Default program in %. The one formatter of these numbers.
"""
# The programs in their order, by their run names, with the names shown (readme-writer project reference > Statuses).
#   Unit [Dict<string, string>]
CONST_NAMES = {'taught': 'Default program', 'optimized': 'Optimized program'}
# Seconds per hour: the throughput is CONST_HOUR / the cycle time.
#   Unit [float] (s)
CONST_HOUR = 3600.0


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
