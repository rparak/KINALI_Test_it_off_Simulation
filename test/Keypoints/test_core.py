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
# File Name: ../test/Keypoints/test_core.py
## =========================================================================== ##

# System (Default)
import math
# Copy (Shallow and deep copy operations)
import copy
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Custom Lib.: the module under test
#   ../Keypoints/Core
from Keypoints.Core import (Check_Keypoint, Check_Keypoints, Format_Check_Table, Pose_Cell, Fixture_State, Resolve_Keypoint,
                            Contact_Group, Keypoints_Error_Cls)
#   ../Configuration/Core, ../Collision/Core, ../Kinematics/Core
from Configuration.Core import Read_PCB_Meta
from Collision.Core import Build_Planning_Model, Robot_Collision_Cls, Cup_Contacts
from Kinematics.Core import Robot_Kinematics_Cls, PCB_For_TCP, TCP_For_PCB

"""
Description:
    Initialization of constants.

    The keypoint check, posing and resolution (SPEC §2 item 3, FEATURES.md F6). The limits are the owner's
    (0.5 mm on the recorded fork path, 3 mm clearance) and SPEC §4.7 (the 1.5 mm press), written out here.
"""
# The fork keypoints posed in their recorded / settled MG06 state (C1).
#   Unit [List<string>]
CONST_FORK = ['fork_start', 'fork_closed_end', 'fork_bar_centred', 'fork_out_of_bar', 'fork_retreat_up', 'fork_open_join']


@pytest.fixture(scope='module')
def planning_model(cell_configuration):
    return Build_Planning_Model(cell_configuration)


@pytest.fixture(scope='module')
def tools(planning_model):
    collision = Robot_Collision_Cls(planning_model)
    return Robot_Kinematics_Cls(planning_model, collision.Data), collision, Read_PCB_Meta()


@pytest.fixture(scope='module')
def results(tools, keypoints, cell_configuration):
    kinematics, collision, meta = tools
    return {r.Name: r for r in Check_Keypoints(kinematics, collision, keypoints, cell_configuration, meta)}


def test_every_keypoint_passes_the_check(results):
    """
    Description:
        All 23 keypoints (SPEC §4.8's 22 and the program's input_pcb_clear) pass; via_mg06_high only warns (wrist_2 at
        -0.94 deg, the family switch within the 1 deg margin, SPEC §4.7).
    """

    verdicts = {name: r.Verdict for name, r in results.items()}
    assert len(verdicts) == 23 and {n for n, v in verdicts.items() if v == 'FAIL'} == set()
    assert {n for n, v in verdicts.items() if v == 'WARN'} == {'via_mg06_high'}


def test_every_keypoint_reaches_its_check_value(results):
    """
    Description:
        FK of every keypoint is within 0.1 mm of its Check position (SPEC §4.8's rounding).
    """

    assert max(r.Check_Error for r in results.values()) <= 0.0001


def test_every_family_matches_wrist_2(results, keypoints):
    """
    Description:
        Every keypoint's family is the sign of its wrist_2 (wrist_up > 0, wrist_down < 0), except within the 1 deg
        margin of the switch.
    """

    for name, keypoint in keypoints.Robot.items():
        wrist_2 = math.degrees(keypoint.Joints[4])
        assert results[name].Family_Ok and (abs(wrist_2) < 1.0 or (wrist_2 > 0) == (keypoint.Family == 'wrist_up')), name


def test_the_relative_keypoints_are_not_stale(results):
    """
    Description:
        Every Relative_To keypoint (11 of SPEC §4.8 and input_pcb_clear) reaches its target within 0.05 mm / 0.01 deg
        (measured at most 24 um).
    """

    relative = [r for r in results.values() if r.Stale_Position is not None]
    assert len(relative) == 12 and max(r.Stale_Position for r in relative) <= 0.00005
    assert max(r.Stale_Angle for r in relative) <= math.radians(0.01)


def test_every_declared_contact_is_really_present(results, keypoints):
    """
    Description:
        Allow_Contact allows only what the keypoint really has (owner, C1): every declared group is found within
        the clearance, the press keypoints 1.48 mm deep (the 1.5 mm press), the fork on the bar at most 0.5 mm.
    """

    for name, keypoint in keypoints.Robot.items():
        assert sorted(keypoint.Allow_Contact) == [g for g, _ in results[name].Allowed], name
    for name in ('input_pcb_press', 'loader_take_press'):
        assert dict(results[name].Allowed)['cups_pcb'] == pytest.approx(-0.0015, abs=0.00005)


@pytest.mark.parametrize('name', CONST_FORK)
def test_the_fork_keypoints_in_their_recorded_state_touch_the_bar_at_most_half_a_millimetre(tools, keypoints, cell_configuration, name):
    """
    Description:
        Posed with MG06 as on the recorded fork path (or settled closed), the fork is at most 0.5 mm into the handle
        bar (C1: fork_closed_end 0.33 mm, fork_open_join +0.08 mm, fork_bar_centred +3.93 mm).
    """

    kinematics, collision, meta = tools
    Pose_Cell(kinematics, keypoints, cell_configuration, meta, name)
    tool, handle = kinematics.Model.body('aubo/tool/ee_tool').id, kinematics.Model.body('mg06_handle').id
    pairs = [p for p in collision.Close_Pairs({tool}, 0.01, kinematics.Data) if p[2] == 'mg06_handle']
    assert all(p[0] >= -0.0005 for p in pairs), pairs[:1]


def test_fork_closed_end_in_the_wrong_mg06_state_is_reported(tools, keypoints, cell_configuration):
    """
    Description:
        The fork keypoint's MG06 state matters (C1): fork_closed_end posed with MG06 settled closed (handle 28.65 deg,
        not the recorded 29.13 deg) puts the bar 5.95 mm into the fork - the check fails on the allowed depth. With
        MG06 open the handle is rotated away and the fork hangs in free space (24.6 mm from MG06's base, measured
        2026-10-24): no contact to report (the owner expected one; FEATURES.md F6, open question).
    """

    kinematics, collision, meta = tools
    closed = Check_Keypoint('fork_closed_end', kinematics, collision, keypoints, cell_configuration, meta, fixture='Closed')
    assert closed.Verdict == 'FAIL' and dict(closed.Allowed)['handle_bar'] == pytest.approx(-0.00595, abs=0.0001)
    assert any('allowed contact handle_bar is 5.95 mm deep' in f for f in closed.Failures)
    opened = Check_Keypoint('fork_closed_end', kinematics, collision, keypoints, cell_configuration, meta, fixture='Open')
    assert opened.Verdict == 'PASS' and opened.Allowed == []


def test_a_colliding_keypoint_is_reported(tools, keypoints, cell_configuration):
    """
    Description:
        home with the shoulder lifted to 1.8 rad (the arm into MG06, test/Collision) fails the clearance, naming the
        upper arm and MG06's base, and the table lists the failure; a joint beyond the model limit fails too.
    """

    kinematics, collision, meta = tools
    broken = copy.deepcopy(keypoints)
    broken.Robot['home'].Joints[1] = 1.8
    result = Check_Keypoint('home', kinematics, collision, broken, cell_configuration, meta)
    assert result.Verdict == 'FAIL' and 'aubo/upper_arm_link' in result.Min_Pair and result.Min_Distance < -0.02
    assert any(line.startswith('  home: ') for line in Format_Check_Table([result], broken))
    broken.Robot['home'].Joints[0] = 3.2
    assert not Check_Keypoint('home', kinematics, collision, broken, cell_configuration, meta).Limits_Ok


def test_a_stale_keypoint_is_reported_and_resolved(tools, keypoints, cell_configuration):
    """
    Description:
        input_pcb_press with its pan joint 0.05 deg off is stale (beyond 0.05 mm), named with the resolver command;
        Resolve_Keypoint returns joints within the tolerance, in the same family; a keypoint without Relative_To
        is refused.
    """

    kinematics, collision, meta = tools
    broken = copy.deepcopy(keypoints)
    broken.Robot['input_pcb_press'].Joints[0] += math.radians(0.05)
    result = Check_Keypoint('input_pcb_press', kinematics, collision, broken, cell_configuration, meta)
    assert result.Stale_Position > 0.00005 and any('resolve_keypoint.py --name input_pcb_press' in f for f in result.Failures)
    broken.Robot['input_pcb_press'].Joints = list(Resolve_Keypoint('input_pcb_press', kinematics, broken, meta))
    again = Check_Keypoint('input_pcb_press', kinematics, collision, broken, cell_configuration, meta)
    assert again.Stale_Position <= 0.00005 and again.Family_Ok
    with pytest.raises(Keypoints_Error_Cls, match='has no Relative_To target'):
        Resolve_Keypoint('home', kinematics, keypoints, meta)


def test_input_pcb_press_poses_the_pcb_in_the_crate_with_four_cups_touching(cell_model, keypoints, cell_configuration):
    """
    Description:
        Posed at input_pcb_press (the physics model), the PCB lies in the input crate and all 4 cups touch it within
        the cup-touch threshold (Cell.yaml Simulation.Cup_Touch).
    """

    kinematics = Robot_Kinematics_Cls(cell_model)
    Pose_Cell(kinematics, keypoints, cell_configuration, Read_PCB_Meta(), 'input_pcb_press')
    m, d = cell_model, kinematics.Data
    assert d.xpos[m.body('pcb').id] == pytest.approx([0.15720, -0.75160, 0.0049], abs=1e-6)
    cups = {m.body('aubo/tool/vgc10_cup%d' % i).id for i in range(4)}
    assert Cup_Contacts(m, d, m.body('pcb').id, cups, cell_configuration.Cup_Touch) == 4


def test_posing_sets_loader_jaws_and_mg06(cell_model, keypoints, cell_configuration):
    """
    Description:
        Posing at fork_closed_end sets the loader at park, the jaws open, MG06 at the recorded row 286 with the press
        plate on the model's coupling; overrides replace the context (MG06 closed: -0.5 / 28.65 deg).
    """

    kinematics = Robot_Kinematics_Cls(cell_model)
    a, q = kinematics.Scene_Address, kinematics.Data.qpos
    Pose_Cell(kinematics, keypoints, cell_configuration, Read_PCB_Meta(), 'fork_closed_end')
    assert (q[a['y_axis']], q[a['z_axis']], q[a['jaw_L']]) == (-0.1995, 0.02, 0.035)
    assert math.degrees(q[a['handle_hinge']]) == pytest.approx(29.125260, abs=1e-6)
    assert q[a['press_slide']] == pytest.approx(0.0104554 * q[a['handle_hinge']] ** 3, rel=1e-9)
    Pose_Cell(kinematics, keypoints, cell_configuration, Read_PCB_Meta(), 'fork_closed_end', loader='home', fixture='Closed')
    assert q[a['y_axis']] == -0.1 and math.degrees(q[a['lid_hinge']]) == pytest.approx(-0.5)
    with pytest.raises(Keypoints_Error_Cls, match="no keypoint 'nowhere'"):
        Pose_Cell(kinematics, keypoints, cell_configuration, Read_PCB_Meta(), 'nowhere')


def test_pcb_for_tcp_inverts_tcp_for_pcb():
    """
    Description:
        The PCB held at a TCP pose is the PCB the TCP was computed from (the 8.94 deg tilted hold pose).
    """

    meta = Read_PCB_Meta()
    position, quat = PCB_For_TCP(*TCP_For_PCB(np.array(meta.Hold_Pos), np.array(meta.Quat)))
    assert position == pytest.approx(meta.Hold_Pos, abs=1e-12) and quat == pytest.approx(meta.Quat, abs=1e-12)


def test_a_wrong_family_is_reported(tools, keypoints, cell_configuration):
    """
    Description:
        home declared wrist_down (its wrist_2 is +90 deg) fails the family rule, naming the family and the angle.
    """

    kinematics, collision, meta = tools
    broken = copy.deepcopy(keypoints)
    broken.Robot['home'].Family = 'wrist_down'
    result = Check_Keypoint('home', kinematics, collision, broken, cell_configuration, meta)
    assert not result.Family_Ok and result.Verdict == 'FAIL'
    assert 'family wrist_down, but wrist_2 is 90.00 deg' in result.Failures


def test_the_contact_groups_of_a_pair(planning_model):
    """
    Description:
        The groups (owner, 2026-10-24): a cup or the cup block on the PCB is cups_pcb; the fork on the handle handle_bar;
        the PCB on a crate geom pcb_crate; the PCB on a table box pcb_crate only while the PCB is inside a crate, otherwise
        no group (the 3 mm clearance applies); the PCB on a jaw pcb_jaws; anything else none.
    """

    from Scene.Core import Cup_Block_Geom
    block = str(Cup_Block_Geom(planning_model))
    assert Contact_Group({'pcb', 'aubo/tool/vgc10_cup2'}, {'pcb_board', '7'}, block, False) == 'cups_pcb'
    assert Contact_Group({'pcb', 'aubo/tool/ee_tool'}, {'pcb_board', block}, block, False) == 'cups_pcb'
    assert Contact_Group({'aubo/tool/ee_tool', 'mg06_handle'}, {'1', '2'}, block, False) == 'handle_bar'
    assert Contact_Group({'pcb', 'workstation'}, {'pcb_board', 'crate_place_0'}, block, False) == 'pcb_crate'
    assert Contact_Group({'pcb', 'workstation'}, {'pcb_board', 'table_right'}, block, True) == 'pcb_crate'
    assert Contact_Group({'pcb', 'workstation'}, {'pcb_board', 'table_right'}, block, False) is None
    assert Contact_Group({'pcb', 'jaw_R'}, {'pcb_board', '412'}, block, False) == 'pcb_jaws'
    assert Contact_Group({'aubo/tool/ee_tool', 'workstation'}, {'1', 'table_left'}, block, True) is None


def test_crate_holding_tells_the_inside_of_a_crate():
    """
    Description:
        A point inside a crate's inner footprint and below its wall top (149 mm) is in that crate; above the walls, or
        between the crates, it is in none.
    """

    from Scene.Core import Crate_Holding
    assert Crate_Holding(np.array([0.15720, -0.75160, 0.0049])) == 'crate_pick'
    assert Crate_Holding(np.array([0.40620, -0.75160, 0.1300])) == 'crate_place'
    assert Crate_Holding(np.array([0.40620, -0.75160, 0.1500])) is None
    assert Crate_Holding(np.array([0.28, -0.75160, 0.0049])) is None
