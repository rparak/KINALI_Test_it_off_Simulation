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
# The home point P_HOME (SPEC §4.8).
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


def test_exact_distances_find_a_near_box_pair_whatever_contacts_are_generated(cell_configuration, constants, simulation):
    """
    Description:
        Posed at P_OUTPUT_PLACE, the carried PCB's board is 1.02 mm above the crate floor. Whether MuJoCo generates a
        contact for it (6 mm margin on the PCB) depends on unrelated geometry: none in the cell with the margin, 16 once
        one more geom - here a box 5 m away - sits on the workstation body (measured 2026-10-24: the workstation body's
        mid-phase, not the box-box collider). Close_Pairs measures the pair exactly in both models (design-decisions).
    """

    import mujoco
    from Configuration.Core import Read_PCB_Meta
    from Keypoints.Core import Pose_Cell
    from Scene.Core import Build_Cell_Spec

    def Model(extra_geom):
        spec = Build_Cell_Spec(cell_configuration)
        if extra_geom:
            box = spec.body('workstation').add_geom()
            box.type, box.size, box.pos, box.contype, box.conaffinity = mujoco.mjtGeom.mjGEOM_BOX, [0.01] * 3, [5, 5, 5], 2, 1
        model = spec.compile()
        for g in range(model.ngeom):
            if model.body(model.geom_bodyid[g]).name.startswith('aubo/') or model.body(model.geom_bodyid[g]).name == 'pcb':
                model.geom_margin[g] = 0.006
        return model

    generated = []
    for extra_geom in (False, True):
        model = Model(extra_geom)
        collision = Robot_Collision_Cls(model)
        Pose_Cell(Robot_Kinematics_Cls(model, collision.Data), constants, simulation, cell_configuration, Read_PCB_Meta(), 'P_OUTPUT_PLACE')
        d, pcb = collision.Data, model.body('pcb').id
        generated.append(len([c for c in d.contact[:d.ncon] if pcb in (model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2])
                              and any((model.geom(g).name or '').startswith('crate_') for g in (c.geom1, c.geom2))]))
        pairs = collision.Close_Pairs({pcb}, 0.003, d, exclude=collision.Robot_Bodies)
        assert pairs and pairs[0][2] == 'workstation' and pairs[0][4].startswith('crate_place') and pairs[0][0] == pytest.approx(0.00102, abs=0.00005)
    assert generated == [0, 16]


def _Brute_Close_Pairs(model, data, bodies, cut, exclude=()):
    """
    Description:
        The exact close pairs by the per-pair loop Close_Pairs used before its vectorised broad phase (git 84e8011):
        the reference the broad phase must reproduce (B2).
    """

    import mujoco
    mine = [g for g in range(model.ngeom) if model.geom_bodyid[g] in bodies and (model.geom_contype[g] or model.geom_conaffinity[g])]
    others = [g for g in range(model.ngeom) if model.geom_bodyid[g] not in bodies and model.geom_bodyid[g] not in exclude
              and (model.geom_contype[g] or model.geom_conaffinity[g])]
    out, ft = [], np.zeros(6)
    for a in mine:
        for b in others:
            if not ((model.geom_contype[a] & model.geom_conaffinity[b]) or (model.geom_contype[b] & model.geom_conaffinity[a])):
                continue
            if np.linalg.norm(data.geom_xpos[a] - data.geom_xpos[b]) - model.geom_rbound[a] - model.geom_rbound[b] > cut:
                continue
            x = mujoco.mj_geomDistance(model, data, a, b, cut, ft)
            if x < cut:
                out.append((x, model.body(model.geom_bodyid[a]).name, model.body(model.geom_bodyid[b]).name,
                            model.geom(a).name or str(a), model.geom(b).name or str(b)))
    return sorted(out)


def test_the_vectorised_broad_phase_gives_the_brute_force_pairs(cell_configuration, constants, simulation, root):
    """
    Description:
        B2: Close_Pairs (vectorised broad phase) returns exactly the brute-force pairs - the same pairs, the same
        distances, so the same minimum - for the robot and for the carried PCB, at every point posed in its scene
        and at 500 random samples of Data/Reference/cell_plan.npz (seed 20261024, the PCB held by the cups), with the
        3 mm clearance and a 50 mm cut.
    """

    import os
    import mujoco
    from Configuration.Core import Read_PCB_Meta
    from Keypoints.Core import Pose_Cell
    from Kinematics.Core import PCB_For_TCP
    model = Build_Planning_Model(cell_configuration)
    collision = Robot_Collision_Cls(model)
    kinematics = Robot_Kinematics_Cls(model, collision.Data)
    robot, pcb = collision.Robot_Bodies, {model.body('pcb').id}
    meta, data = Read_PCB_Meta(), collision.Data

    def Same():
        for cut in (0.003, 0.05):
            for bodies, exclude in ((robot, set()), (pcb, robot)):
                fast = collision.Close_Pairs(bodies, cut, data, exclude=exclude)
                assert fast == _Brute_Close_Pairs(model, data, bodies, cut, exclude)
        return True

    for name in constants.Points:
        Pose_Cell(kinematics, constants, simulation, cell_configuration, meta, name)
        assert Same(), name
    q = np.load(os.path.join(root, 'Data', 'Reference', 'cell_plan.npz'))['q']
    for i in np.random.default_rng(20261024).choice(len(q), 500, replace=False):
        kinematics.Set_Scene(data, pcb=np.r_[PCB_For_TCP(*kinematics.Site_Pose(q[i]))])
        data.qpos[kinematics.Q_Address] = q[i]
        mujoco.mj_kinematics(model, data)
        assert Same(), i


def test_the_table_boxes_are_the_plates_of_the_table_mesh(cell_configuration):
    """
    Description:
        CONST_TABLE_BOXES are the bounding boxes of the two plates of the visual mesh ws_vis_static_217_214_219 (its
        connected components spanning more than 150 mm in x and y and thinner than 50 mm, coincident vertices welded), to 0.5 mm; the
        planning model carries them as boxes on the workstation body with the crates' collision bits (they meet the robot and
        the PCB).
    """

    import mujoco
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    from Scene.Core import Build_Cell_Model
    from Collision.Core import CONST_TABLE_BOXES
    model = Build_Cell_Model(cell_configuration)
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    g = next(i for i in range(model.ngeom) if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH
             and model.mesh(model.geom_dataid[i]).name == 'ws_vis_static_217_214_219')
    mesh = model.geom_dataid[g]
    a, n, fa, fn = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh], model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
    vertices = model.mesh_vert[a:a + n] @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
    _, inverse = np.unique(np.round(vertices, 5), axis=0, return_inverse=True)
    inverse = inverse.ravel()
    faces = inverse[model.mesh_face[fa:fa + fn]]
    rows, cols = np.r_[faces[:, 0], faces[:, 1], faces[:, 2]], np.r_[faces[:, 1], faces[:, 2], faces[:, 0]]
    k, labels = connected_components(coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(inverse.max() + 1,) * 2), directed=False)
    welded = np.zeros((inverse.max() + 1, 3))
    welded[inverse] = vertices
    plates = sorted((tuple(p.min(0)), tuple(p.max(0))) for p in (welded[labels == i] for i in range(k))
                    if np.all(p.max(0)[:2] - p.min(0)[:2] > 0.15) and p.max(0)[2] - p.min(0)[2] < 0.05)
    assert len(plates) == 2
    for (lower, upper), (box_lower, box_upper) in zip(plates, sorted(CONST_TABLE_BOXES.values())):
        assert lower == pytest.approx(box_lower, abs=0.0005) and upper == pytest.approx(box_upper, abs=0.0005)
    planning = Build_Planning_Model(cell_configuration)
    for name, (lower, upper) in CONST_TABLE_BOXES.items():
        box = planning.geom(name)
        assert planning.body(box.bodyid[0]).name == 'workstation' and box.type[0] == mujoco.mjtGeom.mjGEOM_BOX
        assert (box.contype[0], box.conaffinity[0], box.margin[0]) == (2, 1, 0.0)
        assert np.r_[box.size] == pytest.approx((np.array(upper) - lower) / 2)


def test_the_physics_model_has_no_table_collision(cell_model):
    """
    Description:
        The table boxes are in the planning model only: the physics model (whose fingerprint is the baseline) has no
        geom named table_*.
    """

    assert not any((cell_model.geom(g).name or '').startswith('table_') for g in range(cell_model.ngeom))


def test_the_table_is_close_only_to_a_pcb_on_a_crate_floor(cell_configuration, constants, simulation):
    """
    Description:
        Measured 2026-10-24 (FEATURES.md F8): the carried PCB at P_OUTPUT_PLACE is 3.02 mm above the table, and a PCB on a
        crate floor (P_INPUT_PICK, PCB in the input crate) 2.00 mm - the crate floor's thickness.
    """

    from Configuration.Core import Read_PCB_Meta
    from Keypoints.Core import Pose_Cell
    model = Build_Planning_Model(cell_configuration)
    collision = Robot_Collision_Cls(model)
    kinematics = Robot_Kinematics_Cls(model, collision.Data)
    pcb = {model.body('pcb').id}
    for name, expected in (('P_OUTPUT_PLACE', 0.00302), ('P_INPUT_PICK', 0.0020)):
        Pose_Cell(kinematics, constants, simulation, cell_configuration, Read_PCB_Meta(), name)
        table = [p for p in collision.Close_Pairs(pcb, 0.01, collision.Data, exclude=collision.Robot_Bodies) if p[4].startswith('table_')]
        assert min(p[0] for p in table) == pytest.approx(expected, abs=0.00002), name
