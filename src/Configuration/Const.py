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
# File Name: ../Configuration/Const.py
## =========================================================================== ##

# System (Default)
import os

"""
Description:
    Initialization of constants.

    The constants of the Configuration domain that more than one of its modules uses (Core.py, the cell configuration;
    Program_Parser.py, the cell program): the repository's root and its Configuration/ and Data/ folders, built from this
    module's location (src/Configuration/), never from the working directory (SPEC §7.1); the contact groups; the Process
    durations a Test command may name. A constant one module uses stays in that module.
"""
# The repository root: two folders above src/Configuration/.
#   Unit [string]
CONST_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
# The hand-written configuration (SPEC §8).
#   Unit [string]
CONST_CONFIGURATION = os.path.join(CONST_ROOT, 'Configuration')
# Input data by purpose: PCB/ (the PCB poses), Paths/ (recorded robot paths).
#   Unit [string]
CONST_DATA = os.path.join(CONST_ROOT, 'Data')
# The contact groups, a closed set the configuration files share.
#   Unit [Tuple<string>]
CONST_CONTACT_GROUPS = ('cups_pcb', 'handle_bar', 'pcb_crate', 'pcb_jaws')
# The durations of Cell.yaml's Process section a program's Test command may name (Test: Process.<key>).
#   Unit [Tuple<string>]
CONST_PROCESS_KEYS = ('Test_Duration',)
