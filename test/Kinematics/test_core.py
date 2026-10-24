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
# File Name: ../test/Kinematics/test_core.py
## =========================================================================== ##

# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.: the module under test
#   ../Kinematics/Core
from Kinematics.Core import Robot_Kinematics_Cls, Quaternion_To_Matrix, TCP_For_PCB
#   ../Scene/Core
from Scene.Core import PCB_In_Crate

"""
Description:
    Initialization of constants.

    The initial keypoints of SPEC §4.8 (the brief's table, joints in rad to 4 decimals, the TCP or flange
    position in m to 4 decimals), written out here: the rows with a full position.
"""
# name: (joints, 'tcp' or 'flange', position).
#   Unit [Dict<string, Tuple>] (rad, -, m)
CONST_KEYPOINTS = {
    'home': ([0.7896, 0.6910, 1.8333, -0.4286, 1.5708, 0.0042], 'tcp', [0.3000, -0.5200, 0.4000]),
    'above_input_pcb': ([0.4649, -0.1335, 2.0022, 0.5650, 1.5708, -0.3205], 'tcp', [0.1559, -0.7470, 0.1324]),
    'via_input_to_loader': ([0.0465, 0.0565, 1.4637, -0.1952, 1.4994, -0.7367], 'tcp', [0.0164, -0.5608, 0.3958]),
    'above_loader': ([-0.3720, 0.1896, 1.7926, -0.0310, 1.4280, -1.1529], 'tcp', [0.0045, -0.3298, 0.3158]),
    'loader_handover': ([-0.4296, 0.0867, 2.1469, 0.4347, 1.4246, -1.2110], 'tcp', [0.0045, -0.3049, 0.1578]),
    'via_mg06_high': ([-0.0138, 0.2020, 0.7805, -1.6993, -0.0164, 0.4996], 'flange', [0.1251, -0.3218, 0.9331]),
    'fork_start': ([-1.1328, 0.0136, 1.1340, -2.0213, -1.9182, 1.1519], 'flange', [0.0550, -0.1074, 0.7531]),
    'fork_closed_end': ([-0.5157, 0.0227, 2.4098, -0.7545, -1.3011, 1.1518], 'flange', [0.0550, -0.2840, 0.3154]),
    'fork_retreat_up': ([-0.8243, 0.2137, 2.0471, -1.3081, -1.6097, 1.1518], 'flange', [0.0550, -0.2028, 0.4825]),
    'above_output': ([1.0463, 0.0611, 2.2028, 0.5709, 1.5708, 0.2609], 'tcp', [0.4050, -0.7470, 0.1334]),
}
# Position tolerance of the table check: 4-decimal joints move the TCP by up to ~0.2 mm, the printed
# position is rounded to 0.05 mm.
#   Unit [float] (m)
CONST_TABLE_TOLERANCE = 0.0005


@pytest.fixture(scope='module')
def kinematics(cell_model):
    return Robot_Kinematics_Cls(cell_model)


@pytest.mark.parametrize('name', sorted(CONST_KEYPOINTS))
def test_forward_kinematics_reproduces_the_keypoint_table(kinematics, name):
    """
    Description:
        The site pose of each keypoint's joints is the TCP / flange position of SPEC §4.8, so the model's
        frames, tool and base pose are the ones the brief was extracted from.
    """

    q, site, position = CONST_KEYPOINTS[name]
    p, _ = kinematics.Site_Pose(np.array(q), kinematics.Site_TCP if site == 'tcp' else kinematics.Site_Flange)
    assert p == pytest.approx(position, abs=CONST_TABLE_TOLERANCE)


@pytest.mark.parametrize('name', sorted(CONST_KEYPOINTS))
def test_ik_returns_to_each_keypoint_pose_from_a_perturbed_start(kinematics, name):
    """
    Description:
        From the keypoint's joints perturbed by 0.05 rad, IK on the same site reaches its pose within the
        solver tolerance (1e-5 m) and reports success.
    """

    q, site, _ = CONST_KEYPOINTS[name]
    site_id = kinematics.Site_TCP if site == 'tcp' else kinematics.Site_Flange
    p, R = kinematics.Site_Pose(np.array(q), site_id)
    start = np.array(q) + 0.05 * np.array([1, -1, 1, -1, 1, -1])
    q_ik, ok = kinematics.Solve_IK(p, R, start, site=site_id, q_ref=np.array(q), iters=500)
    p_ik, R_ik = kinematics.Site_Pose(q_ik, site_id)
    assert ok and np.linalg.norm(p_ik - p) < 1e-5 and np.allclose(R_ik, R, atol=1e-4)


def test_the_tcp_over_a_pcb_in_the_input_crate(kinematics):
    """
    Description:
        The TCP holding a PCB flat in the input crate sits 7.5 mm above the board centre, offset by the
        cup position (SPEC §4.4); with the 120 mm lift it is above_input_pcb's TCP (SPEC §4.8).
    """

    p, R = TCP_For_PCB(*PCB_In_Crate('crate_pick'))
    assert p == pytest.approx([0.15595, -0.74700, 0.0124], abs=5e-6)
    assert np.allclose(R, np.diag([1.0, -1.0, -1.0]))
    assert p + [0, 0, 0.12] == pytest.approx(CONST_KEYPOINTS['above_input_pcb'][2], abs=CONST_TABLE_TOLERANCE)


def test_a_quaternion_becomes_its_rotation_matrix():
    """
    Description:
        Identity, and 90 deg about Z.
    """

    assert np.allclose(Quaternion_To_Matrix([1, 0, 0, 0]), np.eye(3))
    s = np.sqrt(0.5)
    assert np.allclose(Quaternion_To_Matrix([s, 0, 0, s]), [[0, -1, 0], [1, 0, 0], [0, 0, 1]])


def test_the_scene_joints_are_set(kinematics):
    """
    Description:
        Set_Scene writes the MG06, loader and PCB joints; without a PCB pose the PCB is moved to (0, 3, 3).
    """

    kinematics.Set_Scene(lid=-0.1, handle=0.2, press=0.003, y=0.05, z=0.06, jaw=0.01)
    a, qpos = kinematics.Scene_Address, kinematics.Data.qpos
    assert [qpos[a[n]] for n in ['lid_hinge', 'handle_hinge', 'press_slide', 'y_axis', 'z_axis', 'jaw_L', 'jaw_R']] == \
        pytest.approx([-0.1, 0.2, 0.003, 0.05, 0.06, 0.01, 0.01])
    assert list(qpos[a['pcb_free']:a['pcb_free'] + 3]) == [0, 3, 3]


def test_one_ik_step_is_the_verified_dls_step(kinematics):
    """
    Description:
        Regression pin of the IK algorithm (damping, error terms, null space, clipping): one step from home
        perturbed by 0.05 rad gives the iterate measured 2026-10-24 on the code that regenerated the legacy
        plan to 1.2e-14 rad (FEATURES.md F3.4). A pin, not a bound: it detects any change of the step.
    """

    q = np.array(CONST_KEYPOINTS['home'][0])
    p, R = kinematics.Site_Pose(q)
    q1, ok = kinematics.Solve_IK(p, R, q + 0.05 * np.array([1, -1, 1, -1, 1, -1]), q_ref=q, iters=1)
    assert not ok
    assert q1 == pytest.approx([0.7839464111, 0.6934194356, 1.824233254, -0.4411880932, 1.5703622006, 0.0021724899], abs=1e-9)
