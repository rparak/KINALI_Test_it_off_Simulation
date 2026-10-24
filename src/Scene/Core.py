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
# File Name: ../Scene/Core.py
## =========================================================================== ##

# System (Default)
import os
# XML (ElementTree XML API)
import xml.etree.ElementTree as ET
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Custom Lib.: the repository's files
#   ../Configuration/Core
from Configuration.Core import CONST_MODELS, CONST_SCENE_XML, CONST_CELL_SCENE_XML, CONST_ROBOT_XML, CONST_TOOL_XML

"""
Description:
    Initialization of constants.

    The MG06 cell: workstation + MG06 + 2-axis loader (SCHUNK EGI 040) + AUBO i5 on the robot pedestal
    with the OnRobot VGC10 + fork (the tool that opens and closes MG06), built with MjSpec (SPEC §7.2).
    Frames: everything is in the MG06 fixture frame (origin = centre of the MG06 footprint on the
    table, Z up, front of MG06 = -Y). The robot base stands on the robot plate (deska_robot) = the CAD
    origin of the cell. The values below are the verified physics (SPEC §3): never changed in a style
    or structure change; the compiled model is pinned by its fingerprint (SPEC §10.2).
"""
# Top of deska_robot (= CAD origin of the cell .blend).
#   Unit [Vector<float> 1x3] (m)
CONST_ROBOT_BASE = np.array([0.42072, -0.33497, 0.16200])
# Robot base yaw: all tasks lie within +-60 deg of the pan-joint zero.
#   Unit [float] (rad)
CONST_ROBOT_YAW = np.radians(-135.0)
# The robot joints, base to wrist 3 (prefixed 'aubo/' in the cell).
#   Unit [List<string>]
CONST_ROBOT_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# Joint position servo gains.
#   Unit [List<float>] (Nm/rad, Nm s/rad)
CONST_KP = [5000, 5000, 4000, 2000, 2000, 1500]
CONST_KV = [150, 150, 100, 25, 25, 15]
# Joint torque limits (estimate; AUBO does not publish joint torques).
#   Unit [List<float>] (Nm)
CONST_FMAX = [180, 180, 150, 60, 60, 60]
# Reflected motor + gear inertia (estimate, ratio ~100).
#   Unit [List<float>] (kg m^2)
CONST_ARMATURE = [0.2, 0.2, 0.15, 0.04, 0.04, 0.04]
# AUBO i5 datasheet: joint speed 220 deg/s (J1-J4), 230 deg/s (J5-J6), payload 5 kg, reach 886.5 mm.
#   Unit [Vector<float> 1x6] (rad/s)
CONST_VMAX_DATASHEET = np.radians([220, 220, 220, 220, 230, 230])
# OnRobot VGC10 with 4 silicone cups D15 mm at 40 kPa (datasheet: 20-60 % vacuum recommended):
# theoretical holding force = 4 * pi * 0.0075^2 * 40e3 = 28 N.
#   Unit [float] (m), [int], [float] (Pa)
CONST_CUP_R, CONST_CUP_N, CONST_VACUUM_PA = 0.0075, 4, 40e3
#   Unit [float] (N)
CONST_VACUUM_FORCE = CONST_CUP_N * np.pi * CONST_CUP_R**2 * CONST_VACUUM_PA
# The cup centres in the tool frame.
#   Unit [List<Tuple<float>>] (m)
CONST_CUP_XY = [(0.035, 0.018), (-0.035, 0.018), (0.035, -0.018), (-0.035, -0.018)]
# First-order lag of the loader axis drives.
#   Unit [float] (s)
CONST_LOADER_TAU = 0.03
# Blue crates (Box_Type_A): outer and inner dimensions from the CAD mesh, (x0, y0), (x1, y1).
# crate_pick = left crate (Box_Type_A_ID_2, seen from the front of the cell), crate_place = right crate
# (Box_Type_A_ID_1).
#   Unit [Dict<string, Dict<string, Tuple>>] (m)
CONST_CRATES = {'crate_pick': dict(outer=((0.0587, -0.9001), (0.2557, -0.6031)), inner=((0.0737, -0.8771), (0.2407, -0.6261))),
                'crate_place': dict(outer=((0.3077, -0.9001), (0.5047, -0.6031)), inner=((0.3227, -0.8771), (0.4897, -0.6261)))}
# Crate floor top and wall top.
#   Unit [float] (m)
CONST_CRATE_FLOOR, CONST_CRATE_TOP = 0.004, 0.149
# PCB body frame = plate axes, origin = board centre; the board's half thickness.
#   Unit [float] (m)
CONST_PCB_HALF_T = 0.0009
# The suction TCP in the PCB frame: the cups on the top of the component area (box).
#   Unit [Vector<float> 1x3] (m)
CONST_SUCTION_IN_PCB = np.array([-0.00125, 0.0046, 0.0075])
# The tool axes in the PCB frame: tool +Z = -PCB normal, tool +X = PCB +X.
#   Unit [Matrix<float> 3x3] (-)
CONST_R_TOOL_IN_PCB = np.diag([1.0, -1.0, -1.0])


class Scene_Error_Cls(ValueError):
    """
    Description:
        A model file does not load. The message names the file and MuJoCo's reason; a script prints it
        verbatim as an [ERROR] line.
    """


def Crate_Center(name: str) -> np.ndarray:
    """
    Description:
        The centre of a crate's inner floor.

    Args:
        (1) name [string]: 'crate_pick' (left, input) or 'crate_place' (right, output).

    Returns:
        (1) parameter [Vector<float> 1x2]: x, y (m).
    """

    (x0, y0), (x1, y1) = CONST_CRATES[name]['inner']
    return np.array([(x0 + x1) / 2, (y0 + y1) / 2])


def PCB_In_Crate(name: str) -> tp.Tuple[np.ndarray, np.ndarray]:
    """
    Description:
        The pose of a PCB lying flat on a crate's floor, centred.

    Args:
        (1) name [string]: The crate.

    Returns:
        (1) parameter [Vector<float> 1x3]: The PCB centre (m).
        (2) parameter [Vector<float> 1x4]: Its quaternion, w first (identity).
    """

    c = Crate_Center(name)
    return np.array([c[0], c[1], CONST_CRATE_FLOOR + CONST_PCB_HALF_T]), np.array([1.0, 0, 0, 0])


def _Load_Spec(path: str) -> mujoco.MjSpec:
    """
    Description:
        An MJCF file as an MjSpec; a file that does not load is a Scene_Error_Cls naming it.

    Args:
        (1) path [string]: The file.

    Returns:
        (1) parameter [mujoco.MjSpec]: The spec.
    """

    try:
        return mujoco.MjSpec.from_file(path)
    except ValueError as error:
        raise Scene_Error_Cls('The model file %s does not load (%s). Restore it from git.' % (path, error)) from error


def Build_Cell_Spec() -> mujoco.MjSpec:
    """
    Description:
        The cell as an MjSpec (SPEC §7.2): the scene, the crates' collision as floor + 4 walls, the AUBO
        i5 at its base pose with the VGC10/fork tool rigidly on the flange, one body and one adhesion
        actuator per suction cup, the robot servos, the loader drive lag and the look.

    Returns:
        (1) parameter [mujoco.MjSpec]: The uncompiled cell.
    """

    sc = _Load_Spec(CONST_SCENE_XML)
    for k in list(sc.keys):
        sc.delete(k)
    # 1\ Crates: replace the filled convex hulls by floor + 4 walls (the robot reaches into them).
    for g in list(sc.geoms):
        if g.name.startswith('ws_col_Box_Type_A'):
            sc.delete(g)
    ws = sc.body('workstation')
    for n, c in CONST_CRATES.items():
        (ox0, oy0), (ox1, oy1) = c['outer']
        (ix0, iy0), (ix1, iy1) = c['inner']
        zt = CONST_CRATE_TOP
        boxes = [((ox0, oy0, 0.002), (ox1, oy1, CONST_CRATE_FLOOR)), ((ox0, oy0, 0), (ix0, oy1, zt)), ((ix1, oy0, 0), (ox1, oy1, zt)),
                 ((ix0, oy0, 0), (ix1, iy0, zt)), ((ix0, iy1, 0), (ix1, oy1, zt))]
        for i, (lo, hi) in enumerate(boxes):
            lo, hi = np.array(lo), np.array(hi)
            # Contact bits: the loader, the robot and the PCB touch the crates.
            ws.add_geom(name=f'{n}_{i}', type=mujoco.mjtGeom.mjGEOM_BOX, pos=(lo + hi) / 2, size=(hi - lo) / 2,
                        contype=2, conaffinity=1, group=3, rgba=[.3, .5, .9, .3])
    # 2\ Robot + tool; the tool is rigidly mounted on the flange.
    rb = _Load_Spec(CONST_ROBOT_XML)
    tl = _Load_Spec(CONST_TOOL_XML)
    tool = tl.body('ee_tool')
    for j in list(tool.joints):
        tl.delete(j)
    tool.pos = [0, 0, 0]
    tool.gravcomp = 1
    # 3\ Suction cups: their own (rigid) bodies, so the adhesion actuator acts only on the cups, not on
    #    the fork. The cup block is 2 mm shorter, so the cups stick out.
    for g in tool.geoms:
        if g.type == mujoco.mjtGeom.mjGEOM_BOX and abs(g.size[0] - 0.0525) < 1e-4:
            g.size = [0.0525, 0.0293, 0.038]
            g.pos = [0, 0, 0.2205]
    # Each cup its own (rigid) body + adhesion actuator: a cup holds only if it touches.
    for i, (x, y) in enumerate(CONST_CUP_XY):
        cb = tool.add_body(name=f'vgc10_cup{i}')
        # Bellows of the cup: 3 mm spring travel along the tool axis (7 N compresses it ~2 mm).
        cb.add_joint(name=f'cup{i}_bellows', type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 0, 1], range=[-0.003, 0.0005],
                     stiffness=3000, springref=0, damping=20, armature=0.005)
        cb.add_geom(name=f'cup{i}_mass', type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.004, 0, 0], pos=[x, y, 0.25], mass=0.02,
                    contype=0, conaffinity=0, group=4)
        # Collision as a square pad of the same area (flat face contact: clean normals for the adhesion
        # force); silicone cup contact parameters (SPEC §4.7).
        a = np.sqrt(np.pi) * CONST_CUP_R / 2
        cb.add_geom(name=f'cup{i}', type=mujoco.mjtGeom.mjGEOM_BOX, size=[a, a, 0.00155], pos=[x, y, 0.2606 - 0.00155],
                    rgba=[0.15, 0.15, 0.15, 1], friction=[2.0, 0.02, 0.001], condim=4, group=1,
                    solref=[0.002, 1], solimp=[0.99, 0.999, 0.0005, 0.5, 2], solmix=100,
                    contype=1, conaffinity=1)
    rb.site('attachment_site').attach_body(tool, 'tool/', '')
    fr = sc.worldbody.add_frame(pos=CONST_ROBOT_BASE, quat=[np.cos(CONST_ROBOT_YAW / 2), 0, 0, np.sin(CONST_ROBOT_YAW / 2)])
    fr.attach_body(rb.body('base_link'), 'aubo/', '')
    # 4\ Robot + tool touch MG06, the loader, the PCB, the workstation (bit 2) and the crates.
    for b in sc.bodies:
        if b.name.startswith('aubo/'):
            b.gravcomp = 1
            for g in b.geoms:
                if g.contype or g.conaffinity:
                    g.contype, g.conaffinity = 1, 3
    # Parts mounted together never collide (MuJoCo excludes only direct parent/child pairs).
    for b1, b2 in [('tool/ee_tool', 'wrist_3_link'), ('tool/ee_tool', 'wrist_2_link'), ('wrist_3_link', 'wrist_1_link')]:
        sc.add_exclude(bodyname1='aubo/' + b1, bodyname2='aubo/' + b2)
    # 5\ Joint position servos with armature and damping.
    for jn, kp, kv, fm, arm in zip(CONST_ROBOT_JOINTS, CONST_KP, CONST_KV, CONST_FMAX, CONST_ARMATURE):
        j = sc.joint('aubo/' + jn)
        j.armature = arm
        j.damping = np.array([2.0, 0, 0])
        a = sc.add_actuator(name='robot_' + jn, target='aubo/' + jn, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.set_to_position(kp=kp, kv=kv)
        a.forcerange = [-fm, fm]
        a.forcelimited = True
        a.ctrlrange = [-3.04, 3.04]
        a.ctrllimited = True
    # 6\ Vacuum: an adhesion actuator on each cup - pulls only what the cup actually touches (ctrl 0..1 =
    #    off..on).
    for i in range(CONST_CUP_N):
        va = sc.add_actuator(name=f'vacuum{i}', target=f'aubo/tool/vgc10_cup{i}', trntype=mujoco.mjtTrn.mjTRN_BODY)
        va.set_to_adhesion(gain=CONST_VACUUM_FORCE / CONST_CUP_N)
        va.ctrlrange = [0, 1]
        va.ctrllimited = True
    # 7\ Loader axis drives: first-order lag between command and servo set-point.
    for n in ('y_servo', 'z_servo'):
        a = sc.actuator(n)
        a.dyntype = mujoco.mjtDyn.mjDYN_FILTEREXACT
        a.dynprm[0] = CONST_LOADER_TAU
    _Apply_Look(sc)
    return sc


def _Apply_Look(sc: mujoco.MjSpec) -> None:
    """
    Description:
        The look: a neutral floor, three lights with shadows, metal / plastic materials, cameras on the
        PCB and the tool. The PCB keeps its own colour (no material on pcb* geoms).

    Args:
        (1) sc [mujoco.MjSpec]: The cell, changed in place.
    """

    sc.visual.quality.shadowsize = 8192
    sc.visual.headlight.ambient = [0.25, 0.25, 0.25]
    sc.visual.headlight.diffuse = [0.35, 0.35, 0.35]
    t = sc.texture('grid')
    t.builtin = mujoco.mjtBuiltin.mjBUILTIN_FLAT
    t.mark = mujoco.mjtMark.mjMARK_RANDOM
    t.random = 0.25
    t.rgb1 = [0.62, 0.62, 0.60]
    t.rgb2 = [0.62, 0.62, 0.60]
    t.markrgb = [0.55, 0.55, 0.53]
    t.width = t.height = 1024
    mf = sc.material('grid')
    mf.texrepeat = [6, 6]
    mf.reflectance = 0.0
    mf.specular = 0.1
    for light in list(sc.worldbody.lights):
        sc.delete(light)
    for pos, d, dif, sh in [([1.2, -2.0, 2.6], [-0.4, 0.7, -1.0], 0.55, True), ([-1.5, -1.0, 2.0], [0.6, 0.4, -1.0], 0.30, True),
                            ([0.0, 1.5, 2.2], [0.0, -0.6, -1.0], 0.20, False)]:
        sc.worldbody.add_light(pos=pos, dir=d / np.linalg.norm(d), diffuse=[dif] * 3, specular=[0.25] * 3, castshadow=sh,
                               type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
    materials = {'alu': dict(specular=0.75, shininess=0.65), 'plastic': dict(specular=0.25, shininess=0.3),
                 'dark': dict(specular=0.15, shininess=0.2)}
    for n, kw in materials.items():
        sc.add_material(name='look_' + n, rgba=[1, 1, 1, 1], **kw)
    for g in sc.geoms:
        if g.group != 2 or g.material or g.name.startswith('pcb'):
            continue
        r, gg, b = g.rgba[:3]
        mx, mn = max(r, gg, b), min(r, gg, b)
        g.material = 'look_dark' if mx < 0.25 else ('look_alu' if mx - mn < 0.18 else 'look_plastic')
    sc.worldbody.add_camera(name='cam_pcb', pos=[-0.55, -0.85, 0.75], mode=mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY,
                            targetbody='pcb', fovy=40)
    sc.worldbody.add_camera(name='cam_tool', pos=[-0.30, 0.35, 0.80], mode=mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY,
                            targetbody='aubo/tool/ee_tool', fovy=45)


def Build_Cell_Model() -> mujoco.MjModel:
    """
    Description:
        The compiled cell (SPEC §4.3).

    Returns:
        (1) parameter [mujoco.MjModel]: The model.
    """

    return Build_Cell_Spec().compile()


def Export_Cell_Scene(path: str = CONST_CELL_SCENE_XML) -> str:
    """
    Description:
        Write the assembled cell as one MJCF for MuJoCo's viewer, with the mesh paths relative to the
        file's folder, so it loads from any working directory.

    Args:
        (1) path [string]: The file (default models/cell_scene.xml).

    Returns:
        (1) parameter [string]: The absolute path written.
    """

    path = os.path.abspath(path)
    sc = Build_Cell_Spec()
    sc.compile()
    r = ET.fromstring(sc.to_xml())
    for c in r.iter('compiler'):
        c.set('meshdir', os.path.relpath(CONST_MODELS, os.path.dirname(path)))
    for me in r.iter('mesh'):
        n, f = me.get('name', ''), me.get('file', '')
        if n.startswith('aubo/tool/'):
            me.set('file', 'robot_tool/assets/' + f)
        elif n.startswith('aubo/'):
            me.set('file', 'aubo_i5/meshes/' + f)
    ET.ElementTree(r).write(path)
    return path
