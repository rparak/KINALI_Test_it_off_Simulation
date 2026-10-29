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
# File Name: ../test/Example/Cell/test_results.py
## =========================================================================== ##

# System (Default)
import os
# Importlib (The implementation of import)
import importlib.util
# Pytest (Testing framework) [pip3 install pytest]
import pytest

"""
Description:
    Example/Cell/results.py: the comparison table of cell.py run.
"""
# The module.
#   Unit [string]
CONST_MODULE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'Example', 'Cell', 'results.py')


@pytest.fixture(scope='module')
def results():
    """
    Description:
        Example/Cell/results.py imported as a module.
    """

    spec = importlib.util.spec_from_file_location('results', CONST_MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_table_of_the_v1_programs(results):
    """
    Description:
        The v1.0 cycle times 70.404 s and 59.148 s (SPEC §10.2's baselines): throughputs 51.1 and 60.9 PCB/h (3600 s / cycle,
        to 0.1), the change -16.0 % and +19.0 %, 28/28 pass in both rows; a program with an unmet check is marked FAIL.
    """

    lines = results.Table_Lines({'taught': 70.404, 'optimized': 59.148}, {'taught': (28, 28), 'optimized': (28, 28)})
    assert lines == ['program               cycle [s]  throughput [PCB/h]  acceptance',
                     'Default program          70.404                51.1  28/28 pass',
                     'Optimized program        59.148                60.9  28/28 pass',
                     'change                  -16.0 %             +19.0 %']
    lines = results.Table_Lines({'taught': 70.404, 'optimized': 59.148}, {'taught': (28, 28), 'optimized': (27, 28)})
    assert lines[2].endswith('27/28 FAIL')


def test_the_change_is_against_the_default_program(results):
    """
    Description:
        Change: 100 (optimized / default - 1); 50 against 100 is -50 %, 150 is +50 %.
    """

    assert results.Change(100.0, 50.0) == pytest.approx(-50.0) and results.Change(100.0, 150.0) == pytest.approx(50.0)
