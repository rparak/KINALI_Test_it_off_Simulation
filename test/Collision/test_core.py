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
# File Name: ../test/Collision/test_core.py
## =========================================================================== ##

# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.: the module under test
#   ../Collision/Core
from Collision.Core import Build_Planning_Model, Robot_Collision_Cls
#   ../Kinematics/Core
from Kinematics.Core import Robot_Kinematics_Cls

"""
Description:
    Initialization of constants.

    Configurations measured 2026-10-24: home is clear of everything within the 3 mm planning clearance;
    home with the shoulder lifted to 1.8 rad drives the upper arm about 21.7 mm into the MG06 base plate.
"""
# The home keypoint (SPEC §4.8).
#   Unit [Vector<float> 1x6] (rad)
CONST_HOME = [0.7896, 0.6910, 1.8333, -0.4286, 1.5708, 0.0042]
# Home with the shoulder lift at 1.8 rad.
#   Unit [Vector<float> 1x6] (rad)
CONST_INTO_MG06 = [0.7896, 1.8, 1.8333, -0.4286, 1.5708, 0.0042]


@pytest.fixture(scope='module')
def planning(cell_configuration):
    model = Build_Planning_Model(cell_configuration)
    collision = Robot_Collision_Cls(model)
    Robot_Kinematics_Cls(model, collision.Data).Set_Scene()
    return model, collision


def test_the_planning_model_has_the_margin_on_robot_and_pcb_geoms_only(planning):
    """
    Description:
        Every robot and PCB geom carries the 6 mm planning margin; no other geom has one.
    """

    model, collision = planning
    for g in range(model.ngeom):
        body = model.geom_bodyid[g]
        expected = 0.006 if body in collision.Robot_Bodies or model.body(body).name == 'pcb' else 0.0
        assert model.geom_margin[g] == pytest.approx(expected), model.geom(g).name


def test_home_has_no_contact_within_the_clearance(planning):
    """
    Description:
        At home (MG06 open, loader home, PCB away) the robot has no contact closer than 3 mm.
    """

    _, collision = planning
    assert collision.Contacts(np.array(CONST_HOME), ignore_pcb=True, margin=-0.003) == []


def test_a_colliding_configuration_is_reported_with_its_pair(planning):
    """
    Description:
        The arm driven into MG06 is reported as contacts, deepest first, naming the upper arm and the MG06
        base, about 21.7 mm deep; Clearance reports a negative distance for it.
    """

    _, collision = planning
    contacts = collision.Contacts(np.array(CONST_INTO_MG06), ignore_pcb=True)
    assert contacts and contacts[0][0] == pytest.approx(-0.0217, abs=0.001)
    assert {contacts[0][1], contacts[0][2]} == {'mg06_base', 'aubo/upper_arm_link'}
    assert [c[0] for c in contacts] == sorted(c[0] for c in contacts)
    # A positive margin reports only deeper penetrations: none is 30 mm deep.
    assert collision.Contacts(np.array(CONST_INTO_MG06), ignore_pcb=True, margin=0.03) == []
    distance, pair = collision.Clearance(np.array(CONST_INTO_MG06))
    assert distance < 0 and pair is not None
