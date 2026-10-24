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
# File Name: ../Collision/Core.py
## =========================================================================== ##

# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Custom Lib.: the cell and the robot's joint names
#   ../Scene/Core
from Scene.Core import Build_Cell_Spec, Compile_Spec, CONST_ROBOT_JOINTS
#   ../Configuration/Core
from Configuration.Core import Cell_Configuration_Str

"""
Description:
    Initialization of constants.

    Collision queries of a robot configuration against the cell (SPEC §7.4): its contacts (penetration
    below a margin) and its clearance to everything else. The planning model carries a 6 mm geom margin
    on the robot and PCB geoms, so contacts are generated within the 3 mm planning clearance, and the table
    top as two boxes (the physics model has no table collision).
"""
# The planning model's geom margin on the robot and PCB geoms, as a multiple of the configured clearance
# (Configuration/Cell.yaml Clearance.Robot), so that MuJoCo generates contacts within the clearance.
#   Unit [float] (-)
CONST_MARGIN_FACTOR = 2.0
# The table top, which models/scene.xml has only as visual meshes: its two plates, the bounding boxes (no margin: a
# plate is a box) of the two large connected components of the visual mesh ws_vis_static_217_214_219 (coincident
# vertices welded; measured 2026-10-24, FEATURES.md F8; the gap between them holds two rails no higher than the top).
# Planning and checking only: never in the physics model.
#   Unit [Dict<string, Tuple<Vector<float> 1x3>>] (name -> lower corner, upper corner, m, MG06 fixture frame)
CONST_TABLE_BOXES = {'table_left': ((-0.21129, -0.86012, -0.038), (-0.01128, 0.18988, 0.002)),
                     'table_right': ((0.02072, -0.86012, -0.038), (0.54072, 0.18988, 0.002))}
# The table boxes' collision bits and group: the crates' (contype 2, conaffinity 1, group 3), which meet both the robot
# and the PCB (the workstation's ws_col_* meshes, 2 / 0, meet the robot only).
#   Unit [int]
CONST_TABLE_CONTYPE, CONST_TABLE_CONAFFINITY, CONST_TABLE_GROUP = 2, 1, 3
# The broad phase's slack: a pair whose bounding spheres are this much farther apart than the cut is still measured,
# so float rounding of the vectorised sphere distance can never drop a pair the exact distance would keep.
#   Unit [float] (m)
CONST_BROAD_SLACK = 1e-9


def Build_Planning_Model(cell: Cell_Configuration_Str) -> mujoco.MjModel:
    """
    Description:
        The cell for collision queries: the cell with the table top as boxes (CONST_TABLE_BOXES, on the workstation
        body) and CONST_MARGIN_FACTOR x the clearance as geom margin on every robot and PCB geom (6 mm for the 3 mm
        clearance). Not for physics (the verified physics has no table collision and no margin on the PCB,
        SPEC §4.7); exact distances (Close_Pairs) ignore the margin.

    Args:
        (1) cell [Cell_Configuration_Str(object)]: The cell configuration.

    Returns:
        (1) parameter [mujoco.MjModel]: The planning model.
    """

    spec = Build_Cell_Spec(cell)
    workstation = spec.body('workstation')
    for name, (lower, upper) in CONST_TABLE_BOXES.items():
        lower, upper = np.array(lower), np.array(upper)
        geom = workstation.add_geom()
        geom.name, geom.type = name, mujoco.mjtGeom.mjGEOM_BOX
        geom.size, geom.pos = (upper - lower) / 2, (upper + lower) / 2
        geom.contype, geom.conaffinity, geom.group = CONST_TABLE_CONTYPE, CONST_TABLE_CONAFFINITY, CONST_TABLE_GROUP
    m = Compile_Spec(spec)
    margin = CONST_MARGIN_FACTOR * cell.Clearance
    robot_bodies = {i for i in range(m.nbody) if m.body(i).name.startswith('aubo/')}
    for g in range(m.ngeom):
        if m.geom_bodyid[g] in robot_bodies or m.body(m.geom_bodyid[g]).name == 'pcb':
            m.geom_margin[g] = margin
    return m


class Robot_Collision_Cls(object):
    """
    Description:
        Contacts and clearance of robot configurations in a given cell model and its current scene state.

    Initialization of the Class:
        Args:
            (1) model [mujoco.MjModel]: The model (usually Build_Planning_Model(cell)).
            (2) data [mujoco.MjData]: Its data (default: a new one).

        Example:
            Initialization:
                # Initialization of the class.
                Cls = Robot_Collision_Cls(Build_Planning_Model(cell))

            Features:
                # Contacts deeper than the clearance, and the closest distance.
                contacts = Cls.Contacts(q, margin=-cell.Clearance)
                distance, pair = Cls.Clearance(q)
    """

    def __init__(self, model: mujoco.MjModel, data: tp.Optional[mujoco.MjData] = None) -> None:
        # << PRIVATE >> #
        self.__model = model
        self.__data = mujoco.MjData(model) if data is None else data
        self.__q_address = np.array([model.jnt_qposadr[model.joint('aubo/' + j).id] for j in CONST_ROBOT_JOINTS])
        self.__robot_bodies = {i for i in range(model.nbody) if model.body(i).name.startswith('aubo/')}
        # The collidable geoms and their bits, bodies and bounding radii, for Close_Pairs' broad phase.
        self.__collidable = np.array([g for g in range(model.ngeom) if model.geom_contype[g] or model.geom_conaffinity[g]])
        self.__geom_body = model.geom_bodyid[self.__collidable]
        self.__geom_type = model.geom_contype[self.__collidable]
        self.__geom_affinity = model.geom_conaffinity[self.__collidable]
        self.__geom_radius = model.geom_rbound[self.__collidable]

    @property
    def Data(self) -> mujoco.MjData:
        """
        Description:
            The default data (share it with a Robot_Kinematics_Cls to query its scene state).

        Returns:
            (1) parameter [mujoco.MjData]: The data used when none is passed.
        """

        return self.__data

    @property
    def Robot_Bodies(self) -> tp.Set[int]:
        """
        Description:
            The ids of the robot's bodies (names starting 'aubo/').

        Returns:
            (1) parameter [Set<int>]: The body ids.
        """

        return set(self.__robot_bodies)

    def Contacts(self, q: np.ndarray, data: tp.Optional[mujoco.MjData] = None, ignore_pcb: bool = False,
                 margin: float = 0.0) -> tp.List[tp.Tuple[float, str, str, str, str]]:
        """
        Description:
            The robot-related contacts of configuration q in the data's current scene state whose
            distance is below -margin (a negative margin reports contacts within that clearance).

        Args:
            (1) q [Vector<float> 1x6]: The joints (rad).
            (2) data [mujoco.MjData]: The data (default: the class's).
            (3) ignore_pcb [bool]: Skip contacts with the PCB (a carried PCB).
            (4) margin [float]: The distance bound (m).

        Returns:
            (1) parameter [List<Tuple>]: (distance, body 1, body 2, geom 1, geom 2), deepest first.
        """

        m = self.__model
        d = self.__data if data is None else data
        d.qpos[self.__q_address] = q
        mujoco.mj_forward(m, d)
        out = []
        for c in d.contact[:d.ncon]:
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if not ({b1, b2} & self.__robot_bodies):
                continue
            names = {m.body(b1).name, m.body(b2).name}
            if ignore_pcb and 'pcb' in names:
                continue
            if c.dist < -margin:
                out.append((c.dist, m.body(b1).name, m.body(b2).name, m.geom(c.geom1).name or str(c.geom1),
                            m.geom(c.geom2).name or str(c.geom2)))
        return sorted(out)

    def Close_Pairs(self, bodies: tp.Set[int], cut: float, data: tp.Optional[mujoco.MjData] = None,
                    exclude: tp.Optional[tp.Set[int]] = None) -> tp.List[tp.Tuple[float, str, str, str, str]]:
        """
        Description:
            Every collidable geom pair with one geom on bodies and the other elsewhere (not on exclude) whose exact
            signed distance (mj_geomDistance; negative = penetration) is below cut, in the data's current
            kinematics. Exact distances, not generated contacts: MuJoCo's box-box collider reports no pair that is
            near but not touching, whatever the geom margin, so contacts alone miss a box 1 mm from another.

        Args:
            (1) bodies [Set<int>]: The bodies whose geoms are measured (e.g. the robot's, or the PCB's).
            (2) cut [float]: The distance bound (m).
            (3) data [mujoco.MjData]: The data, after mj_kinematics or mj_forward (default: the class's).
            (4) exclude [Set<int>]: Bodies left out as the other side (e.g. the robot, for the PCB).

        Returns:
            (1) parameter [List<Tuple>]: (distance, body, other body, geom, other geom), closest first.
        """

        m = self.__model
        d = self.__data if data is None else data
        exclude = set() if exclude is None else exclude
        # Broad phase, vectorised: compatible collision bits and bounding spheres within the cut (plus a slack, so it
        # keeps every pair the per-pair test kept); the exact distance then decides as before.
        in_mine = np.isin(self.__geom_body, list(bodies))
        in_others = ~in_mine & ~np.isin(self.__geom_body, list(exclude))
        a_index, b_index = np.flatnonzero(in_mine), np.flatnonzero(in_others)
        bits = ((self.__geom_type[a_index][:, None] & self.__geom_affinity[b_index][None, :]) != 0) \
            | ((self.__geom_type[b_index][None, :] & self.__geom_affinity[a_index][:, None]) != 0)
        a_geoms, b_geoms = self.__collidable[a_index], self.__collidable[b_index]
        gap = np.sqrt(np.sum((d.geom_xpos[a_geoms][:, None, :] - d.geom_xpos[b_geoms][None, :, :]) ** 2, axis=2)) \
            - self.__geom_radius[a_index][:, None] - self.__geom_radius[b_index][None, :]
        out, ft = [], np.zeros(6)
        for i, j in zip(*np.nonzero(bits & (gap <= cut + CONST_BROAD_SLACK))):
            a, b = int(a_geoms[i]), int(b_geoms[j])
            x = mujoco.mj_geomDistance(m, d, a, b, cut, ft)
            if x < cut:
                out.append((x, m.body(m.geom_bodyid[a]).name, m.body(m.geom_bodyid[b]).name, m.geom(a).name or str(a), m.geom(b).name or str(b)))
        return sorted(out)

    def Clearance(self, q: np.ndarray, data: tp.Optional[mujoco.MjData] = None, cut: float = 0.05,
                  skip: tp.Sequence[str] = ()) -> tp.Tuple[float, tp.Optional[tp.Tuple[str, str]]]:
        """
        Description:
            The smallest distance between the robot's geoms and the other collidable geoms (approximate:
            pairs farther apart than cut by their bounding spheres are not measured).

        Args:
            (1) q [Vector<float> 1x6]: The joints (rad).
            (2) data [mujoco.MjData]: The data (default: the class's).
            (3) cut [float]: The largest distance measured (m).
            (4) skip [List<string>]: Bodies left out.

        Returns:
            (1) parameter [float]: The distance (m), or cut when nothing is closer.
            (2) parameter [Tuple<string>]: (robot body, other geom or body), or None.
        """

        m = self.__model
        d = self.__data if data is None else data
        d.qpos[self.__q_address] = q
        mujoco.mj_kinematics(m, d)
        rob = [g for g in range(m.ngeom) if m.geom_bodyid[g] in self.__robot_bodies and m.geom_contype[g]]
        oth = [g for g in range(m.ngeom) if m.geom_bodyid[g] not in self.__robot_bodies
               and (m.geom_contype[g] or m.geom_conaffinity[g]) and m.body(m.geom_bodyid[g]).name not in skip]
        best, ft = (cut, None), np.zeros(6)
        for a in rob:
            for b in oth:
                if not ((m.geom_contype[a] & m.geom_conaffinity[b]) or (m.geom_contype[b] & m.geom_conaffinity[a])):
                    continue
                if np.linalg.norm(d.geom_xpos[a] - d.geom_xpos[b]) - m.geom_rbound[a] - m.geom_rbound[b] > cut:
                    continue
                x = mujoco.mj_geomDistance(m, d, a, b, cut, ft)
                if x < best[0]:
                    best = (x, (m.body(m.geom_bodyid[a]).name, m.geom(b).name or m.body(m.geom_bodyid[b]).name))
        return best


"""
Description:
    The checked contact categories of a cycle run (SPEC §4.6), in report order, and the bodies they
    are made of.
"""
# The five contact categories every run reports (no contact = pass, SPEC §10.1 item 1).
#   Unit [List<string>]
CONST_COLLISION_CHECKS = ['robot <-> MG06 / workstation / crates', 'robot tool <-> handle (outside closing / opening)',
                          'robot <-> loader', 'loader <-> MG06 / workstation', 'PCB <-> lid / press plate']
# The loader's moving bodies.
#   Unit [Set<string>]
CONST_LOADER_BODIES = {'y_carriage', 'z_carriage', 'jaw_L', 'jaw_R'}
# The MG06 parts the PCB must never touch.
#   Unit [Set<string>]
CONST_MG06_UPPER_BODIES = {'mg06_lid', 'mg06_press', 'mg06_handle'}
# The fork touches the handle bar on purpose only in these phases of a run.
#   Unit [Tuple<string>]
CONST_FORK_PHASES = ('robot: close MG06', 'robot: open MG06')


class Contact_Classifier_Cls(object):
    """
    Description:
        Sorts a run's contacts into the categories of CONST_COLLISION_CHECKS (or None: an expected
        contact) by the bodies of the two geoms; the fork on the handle counts only outside the closing
        and opening phases. Also names geoms and groups the PCB's penetrations.

    Initialization of the Class:
        Args:
            (1) model [mujoco.MjModel]: The compiled cell.

        Example:
            Initialization:
                # Initialization of the class.
                Cls = Contact_Classifier_Cls(model)

            Features:
                # The category of a contact in the current phase.
                category = Cls.Classify(contact.geom1, contact.geom2, phase)
    """

    def __init__(self, model: mujoco.MjModel) -> None:
        # << PRIVATE >> #
        self.__geom_name = [model.geom(g).name or (model.mesh(model.geom_dataid[g]).name if model.geom_dataid[g] >= 0 else str(g))
                            for g in range(model.ngeom)]
        self.__body_name = [model.body(model.geom_bodyid[g]).name for g in range(model.ngeom)]
        self.__cache = {}

    def Geom_Name(self, geom: int) -> str:
        """
        Description:
            A geom's name, else its mesh's name, else its id.

        Args:
            (1) geom [int]: The geom id.

        Returns:
            (1) parameter [string]: The name.
        """

        return self.__geom_name[geom]

    def Body_Name(self, geom: int) -> str:
        """
        Description:
            The name of a geom's body.

        Args:
            (1) geom [int]: The geom id.

        Returns:
            (1) parameter [string]: The body name.
        """

        return self.__body_name[geom]

    def Classify(self, geom_1: int, geom_2: int, phase: str) -> tp.Optional[str]:
        """
        Description:
            The category of a contact between two geoms in a phase of the run.

        Args:
            (1 - 2) geom_1, geom_2 [int]: The geom ids.
            (3) phase [string]: The run's current phase label.

        Returns:
            (1) parameter [string]: A CONST_COLLISION_CHECKS entry, or None for an expected contact.
        """

        key = (geom_1, geom_2)
        if key not in self.__cache:
            s = {self.__body_name[geom_1], self.__body_name[geom_2]}
            robot = any(b.startswith('aubo/') for b in s)
            if robot and 'pcb' in s:
                k = None
            elif s == {'aubo/tool/ee_tool', 'mg06_handle'}:
                k = 'fork'
            elif robot:
                k = 'robot <-> ' + ('loader' if s & CONST_LOADER_BODIES else 'MG06 / workstation / crates')
            elif s & CONST_LOADER_BODIES and 'pcb' not in s:
                k = 'loader <-> MG06 / workstation'
            elif 'pcb' in s and s & CONST_MG06_UPPER_BODIES:
                k = 'PCB <-> lid / press plate'
            else:
                k = None
            self.__cache[key] = k
        k = self.__cache[key]
        if k == 'fork':
            k = None if phase.startswith(CONST_FORK_PHASES) else 'robot tool <-> handle (outside closing / opening)'
        return k


def Penetration_Group(other_body: str) -> str:
    """
    Description:
        The group of a PCB penetration by the body the PCB touches (the report's groups).

    Args:
        (1) other_body [string]: The other body's name.

    Returns:
        (1) parameter [string]: 'crate', 'loader jaws', 'MG06 holder' or the body name.
    """

    if other_body == 'workstation':
        return 'crate'
    if other_body in CONST_LOADER_BODIES:
        return 'loader jaws'
    return 'MG06 holder' if other_body == 'mg06_base' else other_body


def Cup_Contacts(model: mujoco.MjModel, data: mujoco.MjData, pcb_body: int, cup_bodies: tp.Set[int], touch: float) -> int:
    """
    Description:
        How many suction cups touch the PCB (contact distance below touch, Cell.yaml Simulation.Cup_Touch).

    Args:
        (1) model [mujoco.MjModel]: The model.
        (2) data [mujoco.MjData]: The data after a step.
        (3) pcb_body [int]: The PCB's body id.
        (4) cup_bodies [Set<int>]: The cups' body ids.
        (5) touch [float]: The contact distance below which a cup touches (m).

    Returns:
        (1) parameter [int]: The number of cups touching.
    """

    touching = set()
    for c in data.contact[:data.ncon]:
        b = {model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]}
        if pcb_body in b and b & cup_bodies and c.dist < touch:
            touching.add(c.geom1 if model.geom_bodyid[c.geom1] in cup_bodies else c.geom2)
    return len(touching)
