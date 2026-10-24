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

"""
Description:
    Initialization of constants.

    Collision queries of a robot configuration against the cell (SPEC §7.4): its contacts (penetration
    below a margin) and its clearance to everything else. The planning model carries a 6 mm geom margin
    on the robot and PCB geoms, so contacts are generated within the 3 mm planning clearance.
"""
# Planning clearance: a robot closer than 3 mm to anything counts as a collision (SPEC §2 items 3-4).
#   Unit [float] (m)
CONST_PLANNING_CLEARANCE = 0.003
# Geom margin on the robot and PCB geoms of the planning model, so that MuJoCo generates contacts
# within the clearance (twice the clearance).
#   Unit [float] (m)
CONST_PLANNING_MARGIN = 0.006


def Build_Planning_Model() -> mujoco.MjModel:
    """
    Description:
        The cell for collision queries: the compiled cell with CONST_PLANNING_MARGIN on every robot and
        PCB geom. Not for physics (the verified physics uses no margin on the PCB, SPEC §4.7).

    Returns:
        (1) parameter [mujoco.MjModel]: The planning model.
    """

    m = Build_Cell_Model()
    robot_bodies = {i for i in range(m.nbody) if m.body(i).name.startswith('aubo/')}
    for g in range(m.ngeom):
        if m.geom_bodyid[g] in robot_bodies or m.body(m.geom_bodyid[g]).name == 'pcb':
            m.geom_margin[g] = CONST_PLANNING_MARGIN
    return m


class Robot_Collision_Cls(object):
    """
    Description:
        Contacts and clearance of robot configurations in a given cell model and its current scene state.

    Initialization of the Class:
        Args:
            (1) model [mujoco.MjModel]: The model (usually Build_Planning_Model()).
            (2) data [mujoco.MjData]: Its data (default: a new one).

        Example:
            Initialization:
                # Initialization of the class.
                Cls = Robot_Collision_Cls(Build_Planning_Model())

            Features:
                # Contacts deeper than the clearance, and the closest distance.
                contacts = Cls.Contacts(q, margin=-CONST_PLANNING_CLEARANCE)
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
