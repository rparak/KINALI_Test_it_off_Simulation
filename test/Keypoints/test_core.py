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
# Custom Lib.: the module under test
#   ../Keypoints/Core
from Keypoints.Core import (Check_Keypoint, Check_Keypoints, Format_Check_Table, Pose_Cell, Resolve_Keypoint, Contact_Group, Keypoints_Error_Cls,
                            Family)
#   ../Configuration/Core, ../Collision/Core, ../Kinematics/Core
from Configuration.Core import Read_PCB_Meta
from Collision.Core import Build_Planning_Model, Robot_Collision_Cls, Cup_Contacts
from Kinematics.Core import Robot_Kinematics_Cls, PCB_For_TCP, TCP_For_PCB

"""
Description:
    Initialization of constants.

    The point check, posing and resolution (SPEC §2 item 3, §7.7; FEATURES.md F6, F10). The limits are the owner's (0.5 mm on
    the recorded fork path, 3 mm clearance, the Pose to 1e-5 m / 1e-3 deg) and SPEC §4.7 (the 1.5 mm press), written out here.
"""
# The fork points posed in their recorded / settled MG06 state (C1).
#   Unit [List<string>]
CONST_FORK = ['P_MG06_FORK_START', 'P_MG06_CLOSE_END', 'P_MG06_BAR_CENTRE', 'P_MG06_SLOT_EXIT', 'P_MG06_WAIT', 'P_MG06_OPEN_JOIN']
# The derived points: from a Reference (Constants.yaml) or a PCB hold (Simulation.yaml).
#   Unit [List<string>]
CONST_DERIVED = ['P_INPUT_ABOVE', 'P_INPUT_APPROACH', 'P_INPUT_PICK', 'P_INPUT_CLEAR', 'P_LOADER_ABOVE', 'P_LOADER_APPROACH', 'P_LOADER_PLACE',
                 'P_LOADER_PICK', 'P_LOADER_EXIT', 'P_OUTPUT_ABOVE', 'P_OUTPUT_APPROACH', 'P_OUTPUT_PLACE']


@pytest.fixture(scope='module')
def planning_model(cell_configuration):
    return Build_Planning_Model(cell_configuration)


@pytest.fixture(scope='module')
def tools(planning_model):
    collision = Robot_Collision_Cls(planning_model)
    return Robot_Kinematics_Cls(planning_model, collision.Data), collision, Read_PCB_Meta()


@pytest.fixture(scope='module')
def results(tools, constants, simulation, cell_configuration):
    kinematics, collision, meta = tools
    return {r.Name: r for r in Check_Keypoints(kinematics, collision, constants, simulation, cell_configuration, meta)}


def test_every_point_passes_the_check(results):
    """
    Description:
        All 23 points (SPEC §4.8's 22 and the program's P_INPUT_CLEAR) pass; P_VIA_HOME_MG06 only warns (wrist_2 at -0.94 deg,
        the family switch within the 1 deg margin, SPEC §4.7).
    """

    verdicts = {name: r.Verdict for name, r in results.items()}
    assert len(verdicts) == 23 and {n for n, v in verdicts.items() if v == 'FAIL'} == set()
    assert {n for n, v in verdicts.items() if v == 'WARN'} == {'P_VIA_HOME_MG06'}


def test_every_pose_is_fk_of_its_joints(results):
    """
    Description:
        Every point's Pose (the tool frame in the robot base frame, written from FK to 9 decimals) is FK(Joints, Tool) within
        1e-5 m and 1e-3 deg - measured within 1e-8 m.
    """

    assert max(r.Check_Error for r in results.values()) <= 1e-8 and max(r.Check_Angle for r in results.values()) <= math.radians(1e-3)


def test_the_family_is_wrist_2s_sign(results, constants):
    """
    Description:
        The family is derived from wrist_2: wrist_down for the MG06 fork points, P_MG06_ABOVE and P_VIA_HOME_MG06, wrist_up for
        every PCB-handling point.
    """

    down = {n for n, r in results.items() if r.Family == 'wrist_down'}
    assert down == set(CONST_FORK) | {'P_MG06_ABOVE', 'P_MG06_CLOSE_END', 'P_VIA_HOME_MG06'}
    assert all(Family(constants.Points[n].Joints) == r.Family for n, r in results.items())


def test_the_derived_points_are_not_stale(results):
    """
    Description:
        The 12 derived points reach their Reference's pose moved by their Offset (9) or their PCB-hold target (3) within
        0.05 mm / 0.01 deg (measured at most 0.0387 mm, P_LOADER_EXIT); the others have no target.
    """

    derived = {n for n, r in results.items() if r.Stale_Position is not None}
    assert derived == set(CONST_DERIVED)
    assert max(results[n].Stale_Position for n in derived) == pytest.approx(0.0000387, abs=0.000001)
    assert max(results[n].Stale_Angle for n in derived) <= math.radians(0.01)


def test_every_declared_contact_is_really_present(results, simulation):
    """
    Description:
        Contacts allow only what the point really has (owner, C1): every declared group is found within the clearance, the
        press points 1.48 mm deep (the 1.5 mm press), the fork on the bar at most 0.5 mm.
    """

    for name, scene in simulation.Points.items():
        assert sorted(scene.Contacts) == [g for g, _ in results[name].Allowed], name
    for name in ('P_INPUT_PICK', 'P_LOADER_PICK'):
        assert dict(results[name].Allowed)['cups_pcb'] == pytest.approx(-0.0015, abs=0.00005)


@pytest.mark.parametrize('name', CONST_FORK)
def test_the_fork_points_in_their_recorded_state_touch_the_bar_at_most_half_a_millimetre(tools, constants, simulation, cell_configuration, name):
    """
    Description:
        Posed with MG06 as on the recorded fork path (or settled closed), the fork is at most 0.5 mm into the handle bar
        (C1: P_MG06_CLOSE_END 0.33 mm, P_MG06_OPEN_JOIN +0.08 mm, P_MG06_BAR_CENTRE +3.93 mm).
    """

    kinematics, collision, meta = tools
    Pose_Cell(kinematics, constants, simulation, cell_configuration, meta, name)
    tool = kinematics.Model.body('aubo/tool/ee_tool').id
    pairs = [p for p in collision.Close_Pairs({tool}, 0.01, kinematics.Data) if p[2] == 'mg06_handle']
    assert all(p[0] >= -0.0005 for p in pairs), pairs[:1]


def test_the_close_end_in_the_wrong_mg06_state_is_reported(tools, constants, simulation, cell_configuration):
    """
    Description:
        The fork point's MG06 state matters (C1): P_MG06_CLOSE_END posed with MG06 settled closed (handle 28.65 deg, not the
        recorded 29.13 deg) puts the bar 5.95 mm into the fork - the check fails on the allowed depth; with MG06 open the
        handle is rotated away and the fork hangs in free space.
    """

    kinematics, collision, meta = tools
    closed = Check_Keypoint('P_MG06_CLOSE_END', kinematics, collision, constants, simulation, cell_configuration, meta, fixture='Closed')
    assert closed.Verdict == 'FAIL' and dict(closed.Allowed)['handle_bar'] == pytest.approx(-0.00595, abs=0.0001)
    assert any('allowed contact handle_bar is 5.95 mm deep' in f for f in closed.Failures)
    opened = Check_Keypoint('P_MG06_CLOSE_END', kinematics, collision, constants, simulation, cell_configuration, meta, fixture='Open')
    assert opened.Verdict == 'PASS' and opened.Allowed == []


def test_a_colliding_point_is_reported(tools, constants, simulation, cell_configuration):
    """
    Description:
        P_HOME with the shoulder lifted to 1.8 rad (the arm into MG06) fails the clearance, naming the upper arm and MG06's base
        (its Pose fails too: no longer FK of its joints), and the table lists the failures; a joint beyond the limit fails too.
    """

    kinematics, collision, meta = tools
    broken = copy.deepcopy(constants)
    broken.Points['P_HOME'].Joints[1] = 1.8
    result = Check_Keypoint('P_HOME', kinematics, collision, broken, simulation, cell_configuration, meta)
    assert result.Verdict == 'FAIL' and 'aubo/upper_arm_link' in result.Min_Pair and result.Min_Distance < -0.02
    assert any(f.startswith('Pose ') for f in result.Failures) and any(line.startswith('  P_HOME: ') for line in Format_Check_Table([result]))
    broken.Points['P_HOME'].Joints[0] = 3.2
    assert not Check_Keypoint('P_HOME', kinematics, collision, broken, simulation, cell_configuration, meta).Limits_Ok


def test_a_stale_point_is_reported_and_resolved(tools, constants, simulation, cell_configuration):
    """
    Description:
        P_INPUT_ABOVE with its pan joint 0.05 deg off is stale against its Reference (beyond 0.05 mm), named with the resolver
        command; Resolve_Keypoint returns joints within the tolerance, in the same family, and their Pose; a point without a
        target is refused.
    """

    kinematics, collision, meta = tools
    broken = copy.deepcopy(constants)
    broken.Points['P_INPUT_ABOVE'].Joints[0] += math.radians(0.05)
    result = Check_Keypoint('P_INPUT_ABOVE', kinematics, collision, broken, simulation, cell_configuration, meta)
    assert result.Stale_Position > 0.00005
    assert any('Reference target' in f and 'resolve_keypoint.py --name P_INPUT_ABOVE' in f for f in result.Failures)
    joints, pose = Resolve_Keypoint('P_INPUT_ABOVE', kinematics, broken, simulation, meta, cell_configuration)
    broken.Points['P_INPUT_ABOVE'].Joints, broken.Points['P_INPUT_ABOVE'].Pose = list(joints), list(pose)
    again = Check_Keypoint('P_INPUT_ABOVE', kinematics, collision, broken, simulation, cell_configuration, meta)
    assert again.Stale_Position <= 0.00005 and again.Check_Error <= 1e-8 and again.Family == 'wrist_up'
    with pytest.raises(Keypoints_Error_Cls, match='has no Reference or PCB_Hold target'):
        Resolve_Keypoint('P_HOME', kinematics, constants, simulation, meta, cell_configuration)


def test_a_pose_that_is_not_fk_is_reported(tools, constants, simulation, cell_configuration):
    """
    Description:
        P_HOME's Pose 0.02 mm off in x (beyond 1e-5 m) fails, naming the Pose's error and the resolver; its yaw 0.002 deg off
        (beyond 1e-3 deg) fails too.
    """

    kinematics, collision, meta = tools
    broken = copy.deepcopy(constants)
    broken.Points['P_HOME'].Pose[0] += 0.00002
    result = Check_Keypoint('P_HOME', kinematics, collision, broken, simulation, cell_configuration, meta)
    assert result.Verdict == 'FAIL' and any(f.startswith('Pose 0.0200 mm') and 'FK(Joints, T_SUCTION)' in f for f in result.Failures)
    broken = copy.deepcopy(constants)
    broken.Points['P_HOME'].Pose[5] += math.radians(0.002)
    assert Check_Keypoint('P_HOME', kinematics, collision, broken, simulation, cell_configuration, meta).Verdict == 'FAIL'



def test_a_pose_is_compared_with_fk_as_a_rotation(tools, constants, simulation, cell_configuration):
    """
    Description:
        P_INPUT_PICK's Pose (180, 0, 135) deg written as the same rotation in other numbers - (0, 180, -45) deg - still
        passes; its FK reads roll -179.99999... deg, which the shipped 180 matches as a rotation.
    """

    kinematics, collision, meta = tools
    assert Check_Keypoint('P_INPUT_PICK', kinematics, collision, constants, simulation, cell_configuration, meta).Check_Angle < 1e-8
    other = copy.deepcopy(constants)
    other.Points['P_INPUT_PICK'].Pose[3:] = [0.0, math.pi, math.radians(-45.0)]
    result = Check_Keypoint('P_INPUT_PICK', kinematics, collision, other, simulation, cell_configuration, meta)
    assert result.Verdict == 'PASS' and result.Check_Angle < math.radians(1e-3)


def test_the_pick_poses_the_pcb_in_the_crate_with_four_cups_touching(cell_model, constants, simulation, cell_configuration):
    """
    Description:
        Posed at P_INPUT_PICK (the physics model), the PCB lies in the input crate and all 4 cups touch it within the cup-touch
        threshold (Cell.yaml Simulation.Cup_Touch).
    """

    kinematics = Robot_Kinematics_Cls(cell_model)
    Pose_Cell(kinematics, constants, simulation, cell_configuration, Read_PCB_Meta(), 'P_INPUT_PICK')
    m, d = cell_model, kinematics.Data
    assert d.xpos[m.body('pcb').id] == pytest.approx([0.15720, -0.75160, 0.0049], abs=1e-6)
    cups = {m.body('aubo/tool/vgc10_cup%d' % i).id for i in range(4)}
    assert Cup_Contacts(m, d, m.body('pcb').id, cups, cell_configuration.Cup_Touch) == 4


def test_posing_sets_loader_jaws_and_mg06(cell_model, constants, simulation, cell_configuration):
    """
    Description:
        Posing at P_MG06_CLOSE_END sets the loader at L_PARK, the jaws open, MG06 at the recorded row 286 with the press plate on
        the model's coupling; overrides replace the scene (MG06 closed: -0.5 / 28.65 deg); P_LOADER_PICK poses the jaws at
        G_CLAMP's Jaw_Actual (on the PCB), not at its command.
    """

    kinematics = Robot_Kinematics_Cls(cell_model)
    a, q = kinematics.Scene_Address, kinematics.Data.qpos
    Pose_Cell(kinematics, constants, simulation, cell_configuration, Read_PCB_Meta(), 'P_MG06_CLOSE_END')
    assert (q[a['y_axis']], q[a['z_axis']], q[a['jaw_L']]) == (-0.1995, 0.02, 0.035)
    assert math.degrees(q[a['handle_hinge']]) == pytest.approx(29.125260, abs=1e-6)
    assert q[a['press_slide']] == pytest.approx(0.0104554 * q[a['handle_hinge']] ** 3, rel=1e-9)
    Pose_Cell(kinematics, constants, simulation, cell_configuration, Read_PCB_Meta(), 'P_MG06_CLOSE_END', loader='L_HOME', fixture='Closed')
    assert q[a['y_axis']] == -0.1 and math.degrees(q[a['lid_hinge']]) == pytest.approx(-0.5)
    Pose_Cell(kinematics, constants, simulation, cell_configuration, Read_PCB_Meta(), 'P_LOADER_PICK')
    assert q[a['jaw_L']] == 0.017750000000000002
    with pytest.raises(Keypoints_Error_Cls, match="no point 'nowhere'"):
        Pose_Cell(kinematics, constants, simulation, cell_configuration, Read_PCB_Meta(), 'nowhere')


def test_pcb_for_tcp_inverts_tcp_for_pcb():
    """
    Description:
        The PCB held at a TCP pose is the PCB the TCP was computed from (the 8.94 deg tilted hold pose).
    """

    meta = Read_PCB_Meta()
    position, quat = PCB_For_TCP(*TCP_For_PCB(np.array(meta.Hold_Pos), np.array(meta.Quat)))
    assert position == pytest.approx(meta.Hold_Pos, abs=1e-12) and quat == pytest.approx(meta.Quat, abs=1e-12)


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
