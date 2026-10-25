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
# File Name: ../Simulation/Const.py
## =========================================================================== ##

# Description:
#     Initialization of constants.
#
#     The constants of the Simulation domain that more than one of its modules uses (Core.py, the run; Report.py, its
#     acceptance and report). A constant one module uses stays in that module. (A comment, not a docstring: this module
#     imports nothing, so a docstring would be its first statement and read as its header.)

# SPEC §10.1: the cups that must touch at each pick.
#   Unit [int]
CONST_CUPS = 4
# The distance the robot-loader pairs are reported up to.
#   Unit [float] (m)
CONST_LOADER_CUT = 0.05
# The input of the simulated vacuum switch (F11; Simulation.yaml Vacuum_OK_Min_Cups).
#   Unit [string]
CONST_VACUUM_OK = 'DI_VACUUM_OK'

