## =========================================================================== ##
# MIT License
# Copyright (c) 2026 Roman Parak
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
# File Name: ../test/conftest.py
## =========================================================================== ##

# System (Default)
import os
import sys
# Importlib (Import machinery)
import importlib.util
# Pytest (Testing framework) [pip3 install pytest]
import pytest

"""
Description:
    Initialization of constants.

    Shared fixtures. src/ is put on sys.path here (there are no __init__.py files in the RPX layout), and
    MuJoCo's log is routed to logger/, so no MUJOCO_LOG.TXT lands in a test's working directory.
"""
# The repository root.
#   Unit [string]
CONST_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if os.path.join(CONST_ROOT, 'src') not in sys.path:
    sys.path.insert(0, os.path.join(CONST_ROOT, 'src'))
from Configuration.Core import Configure_MuJoCo_Log  # noqa: E402
Configure_MuJoCo_Log()


@pytest.fixture(scope='session')
def root():
    """
    Description:
        The repository root.
    """

    return CONST_ROOT


@pytest.fixture(scope='session')
def cell_model():
    """
    Description:
        The compiled cell (built once per session).
    """

    from Scene.Core import Build_Cell_Model
    return Build_Cell_Model()


def _Run_Script(script, arguments, cwd, timeout=600):
    """
    Description:
        Run Example/<Domain>/<script> the way a person does: a fresh interpreter, from cwd (an unrelated directory).
    """

    import subprocess
    return subprocess.run([sys.executable, os.path.join(CONST_ROOT, 'Example', script)] + list(arguments), cwd=str(cwd),
                          capture_output=True, text=True, timeout=timeout)


@pytest.fixture(scope='session')
def run_script():
    """
    Description:
        The runner of Example/ scripts (_Run_Script).
    """

    return _Run_Script


@pytest.fixture(scope='session')
def fingerprint_tool():
    """
    Description:
        The model fingerprint script imported as a module (a script, not part of src/).
    """

    path = os.path.join(CONST_ROOT, 'Example', 'Scene', 'model_fingerprint.py')
    spec = importlib.util.spec_from_file_location('model_fingerprint', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
