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
from Scene.Core import Build_Cell_Model, CONST_ROBOT_JOINTS
#   ../Configuration/Core
from Configuration.Core import Cell_Configuration_Str

"""
Description:
    Initialization of constants.

    Collision queries of a robot configuration against the cell (SPEC §7.4): its contacts (penetration
    below a margin) and its clearance to everything else. The planning model carries a 6 mm geom margin
    on the robot and PCB geoms, so contacts are generated within the 3 mm planning clearance.
"""
# The planning model's geom margin on the robot and PCB geoms, as a multiple of the configured clearance
# (Configuration/Cell.yaml Clearance.Robot), so that MuJoCo generates contacts within the clearance.
#   Unit [float] (-)
CONST_MARGIN_FACTOR = 2.0


def Build_Planning_Model(cell: Cell_Configuration_Str) -> mujoco.MjModel:
    """
    Description:
        The cell for collision queries: the compiled cell with CONST_MARGIN_FACTOR x the clearance as geom
        margin on every robot and PCB geom (6 mm for the 3 mm clearance). Not for physics (the verified
        physics uses no margin on the PCB, SPEC §4.7).

    Args:
        (1) cell [Cell_Configuration_Str(object)]: The cell configuration.

    Returns:
        (1) parameter [mujoco.MjModel]: The planning model.
    """

    m = Build_Cell_Model(cell)
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
