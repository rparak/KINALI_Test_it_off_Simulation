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
# File Name: ../Scene/Core.py
## =========================================================================== ##

# System (Default)
import os
import sys
# Tempfile (Generate temporary files)
import tempfile
# Warnings (Warning control)
import warnings
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
from Configuration.Core import CONST_MODELS, CONST_SCENE_XML, CONST_CELL_SCENE_XML, CONST_ROBOT_XML, CONST_TOOL_XML, Cell_Configuration_Str

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
# The robot joints, base to wrist 3 (prefixed 'aubo/' in the cell).
#   Unit [List<string>]
# The one known compile warning (KNOWN_ISSUES.md: the vendored robot MJCF's integrator is ignored, the scene runs Euler),
# exactly as MuJoCo 3.16 words it, and the line printed instead.
#   Unit [string]
CONST_KNOWN_ATTACH_WARNING = ("Attach conflict when attaching 'aubo_i5' to 'mg06_loader_scene', policy is 'warning'\n"
                              "integrator: parent has 0 (default), child has 3, keeping parent value")
CONST_KNOWN_ATTACH_NOTE = '[INFO] Known: robot MJCF integrator ignored, scene uses Euler (KNOWN_ISSUES)'
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
# The number of suction cups of the OnRobot VGC10 (the hardware); the vacuum and the cup diameter are in
# Configuration/Cell.yaml, the adhesion force is derived from them.
#   Unit [int]
CONST_CUP_N = 4
# The VGC10's cup block after the build (half sizes): 2 mm shorter than the original so the cups stick out
# (SPEC §4.10); it hangs 2.1 mm above a PCB the cups hold.
#   Unit [List<float>] (m)
CONST_CUP_BLOCK_HALF_SIZE = [0.0525, 0.0293, 0.038]
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
# The studio look (F7): the default NVIDIA Isaac Sim / Isaac Lab environment, approximated (owner, 2026-10-24).
# Visual only; nothing here reaches the physics (SPEC §4.7 Look, design-decisions).
# The sky: a gradient from the zenith (rgb1, darker) to the nadir (rgb2). MuJoCo puts the mean of the two at the
# horizon (measured 2026-10-24), so the nadir is 2 x horizon - zenith; it is below the floor and never seen.
#   Unit [List<float>] (rgb 0-1)
CONST_LOOK_SKY_ZENITH = [0.20, 0.22, 0.25]
CONST_LOOK_HORIZON = [0.42, 0.45, 0.49]
CONST_LOOK_SKY_NADIR = [0.64, 0.68, 0.73]
# The floor grid: base and line colours.
#   Unit [List<float>] (rgb 0-1)
CONST_LOOK_FLOOR_RGB, CONST_LOOK_GRID_RGB = [0.23, 0.24, 0.26], [0.40, 0.42, 0.45]
# The grid texture's size (one line at its edge) and its repeat per unit length (texuniform): MuJoCo maps one
# texture onto 2 m of a plane at texrepeat 1 (measured: lines 2 m apart), so 2 gives a line every 1 m on the world
# axes.
#   Unit [int] (px), [List<float>] (-)
CONST_LOOK_GRID_PX, CONST_LOOK_GRID_REPEAT = 200, [2, 2]
# The floor material: a subtle reflection (owner: 0.05; 0.1 mirrored the cabinet clearly on the dark floor).
#   Unit [float] (-)
CONST_LOOK_FLOOR_REFLECTANCE, CONST_LOOK_FLOOR_SPECULAR, CONST_LOOK_FLOOR_SHININESS = 0.05, 0.2, 0.3
# The floor plane's half sizes: far enough that the plane ends inside the fog (fog end = 6 x extent = 15.6 m).
#   Unit [List<float>] (m)
CONST_LOOK_FLOOR_SIZE = [20, 20, 0.05]
# Fog and haze: start and end as multiples of the model extent (MuJoCo's unit for visual.map), the haze proportion.
#   Unit [float] (-)
CONST_LOOK_FOG_START, CONST_LOOK_FOG_END, CONST_LOOK_HAZE = 1.5, 6.0, 0.6
# The headlight.
#   Unit [List<float>] (rgb 0-1)
CONST_LOOK_HEADLIGHT = dict(ambient=[0.35, 0.36, 0.38], diffuse=[0.25, 0.25, 0.25], specular=[0.05, 0.05, 0.05])
# The key light (a spot above the cell, casting shadows) and the weak fill (directional, no shadows); MuJoCo's
# default light specular (0.3) for both.
#   Unit [Dict] (pos m, dir -, cutoff deg, diffuse rgb 0-1)
CONST_LOOK_KEY_LIGHT = dict(pos=[1.6, 1.2, 4.0], dir=[-0.4, -0.3, -1.0], cutoff=75.0, diffuse=[0.65, 0.64, 0.62])
CONST_LOOK_FILL_LIGHT = dict(dir=[0.5, 0.4, -0.6], diffuse=[0.18, 0.19, 0.21])
# Shadow map size and offscreen multisamples.
#   Unit [int] (px, -)
CONST_LOOK_SHADOW_SIZE, CONST_LOOK_OFFSAMPLES = 8192, 8
# The model statistic: the centre of the cell's bounding box at the default qpos, robot included (x -0.308..0.664,
# y -1.001..0.295, z -0.873..1.290, half diagonal r = 1.351 m), and the extent that fits that sphere into the
# default free camera with a 10 % margin: distance 1.5 x extent >= 1.1 r / sin(fovy 45 deg / 2) = 3.88 m. It also
# keeps the 20 m floor out of the automatic extent and scales the fog (measured 2026-10-24).
#   Unit [List<float>] (m), [float] (m)
CONST_LOOK_CENTER, CONST_LOOK_EXTENT = [0.178, -0.353, 0.208], 2.6


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


def Crate_Holding(position: np.ndarray) -> tp.Optional[str]:
    """
    Description:
        The crate whose inside holds a point: within its inner footprint and below its wall top.

    Args:
        (1) position [Vector<float> 1x3]: The point (m), e.g. the PCB centre.

    Returns:
        (1) parameter [string]: 'crate_pick' or 'crate_place', or None outside both.
    """

    for name, crate in CONST_CRATES.items():
        (x0, y0), (x1, y1) = crate['inner']
        if x0 <= position[0] <= x1 and y0 <= position[1] <= y1 and position[2] <= CONST_CRATE_TOP:
            return name
    return None


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


def Build_Cell_Spec(cell: Cell_Configuration_Str) -> mujoco.MjSpec:
    """
    Description:
        The cell as an MjSpec (SPEC §7.2): the scene, the crates' collision as floor + 4 walls, the AUBO
        i5 at its base pose with the VGC10/fork tool rigidly on the flange, one body and one adhesion
        actuator per suction cup, the robot servos, the loader drive lag and the look.

    Args:
        (1) cell [Cell_Configuration_Str(object)]: The cell configuration (the base pose, the vacuum).

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
    # The cup radius and the adhesion force per cup, derived from the configured vacuum (SPEC §4.3).
    cup_r = cell.Cup_Diameter / 2
    vacuum_force = CONST_CUP_N * np.pi * cup_r**2 * cell.Vacuum_Pressure
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
            g.size = CONST_CUP_BLOCK_HALF_SIZE
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
        a = np.sqrt(np.pi) * cup_r / 2
        cb.add_geom(name=f'cup{i}', type=mujoco.mjtGeom.mjGEOM_BOX, size=[a, a, 0.00155], pos=[x, y, 0.2606 - 0.00155],
                    rgba=[0.15, 0.15, 0.15, 1], friction=[2.0, 0.02, 0.001], condim=4, group=1,
                    solref=[0.002, 1], solimp=[0.99, 0.999, 0.0005, 0.5, 2], solmix=100,
                    contype=1, conaffinity=1)
    rb.site('attachment_site').attach_body(tool, 'tool/', '')
    base, yaw = np.array(cell.Base_Position), cell.Base_Yaw
    fr = sc.worldbody.add_frame(pos=base, quat=[np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
    # MuJoCo prints the known integrator conflict here; Compile_Spec reports it as one line.
    with _Known_Warning_Filter_Cls():
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
        va.set_to_adhesion(gain=vacuum_force / CONST_CUP_N)
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
        The look: the studio look (a grey sky, a dark 1 m grid floor fading into the fog, a key spot light with
        shadows and a weak fill), metal / plastic materials, cameras on the PCB and the tool. The PCB keeps its own
        colour (no material on pcb* geoms). Fog is a render flag, set by Render.Core.

    Args:
        (1) sc [mujoco.MjSpec]: The cell, changed in place.
    """

    sc.visual.quality.shadowsize = CONST_LOOK_SHADOW_SIZE
    sc.visual.quality.offsamples = CONST_LOOK_OFFSAMPLES
    for k, v in CONST_LOOK_HEADLIGHT.items():
        setattr(sc.visual.headlight, k, v)
    sc.visual.rgba.haze = sc.visual.rgba.fog = CONST_LOOK_HORIZON + [1.0]
    sc.visual.map.fogstart, sc.visual.map.fogend, sc.visual.map.haze = CONST_LOOK_FOG_START, CONST_LOOK_FOG_END, CONST_LOOK_HAZE
    sc.stat.center, sc.stat.extent = CONST_LOOK_CENTER, CONST_LOOK_EXTENT
    sky = sc.texture('sky')
    sky.rgb1, sky.rgb2 = CONST_LOOK_SKY_ZENITH, CONST_LOOK_SKY_NADIR
    t = sc.texture('grid')
    t.builtin = mujoco.mjtBuiltin.mjBUILTIN_FLAT
    t.mark = mujoco.mjtMark.mjMARK_EDGE
    t.rgb1 = t.rgb2 = CONST_LOOK_FLOOR_RGB
    t.markrgb = CONST_LOOK_GRID_RGB
    t.width = t.height = CONST_LOOK_GRID_PX
    mf = sc.material('grid')
    mf.texrepeat = CONST_LOOK_GRID_REPEAT
    mf.texuniform = True
    mf.reflectance = CONST_LOOK_FLOOR_REFLECTANCE
    mf.specular = CONST_LOOK_FLOOR_SPECULAR
    mf.shininess = CONST_LOOK_FLOOR_SHININESS
    sc.geom('floor').size = CONST_LOOK_FLOOR_SIZE
    for light in list(sc.worldbody.lights):
        sc.delete(light)
    key, fill = CONST_LOOK_KEY_LIGHT, CONST_LOOK_FILL_LIGHT
    sc.worldbody.add_light(name='key_light', pos=key['pos'], dir=np.array(key['dir']) / np.linalg.norm(key['dir']), cutoff=key['cutoff'],
                           diffuse=key['diffuse'], castshadow=True, type=mujoco.mjtLightType.mjLIGHT_SPOT)
    sc.worldbody.add_light(name='fill_light', dir=np.array(fill['dir']) / np.linalg.norm(fill['dir']), diffuse=fill['diffuse'],
                           castshadow=False, type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
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


def Cup_Block_Geom(model: mujoco.MjModel) -> int:
    """
    Description:
        The cup block's collision geom in a compiled cell: the box on the tool body with the block's half sizes.

    Args:
        (1) model [mujoco.MjModel]: The compiled cell.

    Returns:
        (1) parameter [int]: The geom id.
    """

    tool = model.body('aubo/tool/ee_tool').id
    return next(g for g in range(model.ngeom) if model.geom_bodyid[g] == tool and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX
                and np.allclose(model.geom_size[g], CONST_CUP_BLOCK_HALF_SIZE) and (model.geom_contype[g] or model.geom_conaffinity[g]))


def Build_Cell_Model(cell: Cell_Configuration_Str) -> mujoco.MjModel:
    """
    Description:
        The compiled cell (SPEC §4.3).

    Args:
        (1) cell [Cell_Configuration_Str(object)]: The cell configuration.

    Returns:
        (1) parameter [mujoco.MjModel]: The model.
    """

    return Compile_Spec(Build_Cell_Spec(cell))


class _Known_Warning_Filter_Cls(object):
    """
    Description:
        A context that captures what MuJoCo prints on stderr (file descriptor 2) and the Python warnings raised inside it,
        removes the known warning (CONST_KNOWN_ATTACH_WARNING) and passes everything else on unchanged at its end.

    Initialization of the Class:
        Example:
            Initialization:
                # Assignment of the variables.
                with _Known_Warning_Filter_Cls() as known:
                    model = spec.compile()

            Features:
                # Properties of the class.
                known.Seen
    """

    def __init__(self) -> None:
        self.Seen = False

    def __enter__(self) -> '_Known_Warning_Filter_Cls':
        """
        Description:
            Start capturing: stderr (file descriptor 2) into a temporary file, the Python warnings into a list.

        Returns:
            (1) parameter [_Known_Warning_Filter_Cls(object)]: The filter.
        """

        sys.stderr.flush()
        self._Saved, self._Captured = os.dup(2), tempfile.TemporaryFile(mode='w+b')
        self._Warnings = warnings.catch_warnings(record=True)
        self._Caught = self._Warnings.__enter__()
        warnings.simplefilter('always')
        os.dup2(self._Captured.fileno(), 2)
        return self

    def __exit__(self, *exc: tp.Any) -> None:
        """
        Description:
            Stop capturing; pass on everything but the known warning (Seen tells whether it came).

        Args:
            (1) exc [Any]: The exception, if any (not suppressed).
        """

        os.dup2(self._Saved, 2)
        os.close(self._Saved)
        self._Warnings.__exit__(None, None, None)
        self._Captured.seek(0)
        printed = self._Captured.read().decode('utf-8', 'replace')
        self._Captured.close()
        known = 'WARNING: %s\n' % CONST_KNOWN_ATTACH_WARNING
        rest = printed.replace(known + '\n', '', 1) if known + '\n' in printed else printed.replace(known, '', 1)
        self.Seen = rest != printed
        if rest:
            sys.stderr.write(rest)
            sys.stderr.flush()
        for warning in self._Caught:
            if str(warning.message) == CONST_KNOWN_ATTACH_WARNING:
                self.Seen = True
            else:
                warnings.warn_explicit(warning.message, warning.category, warning.filename, warning.lineno)


def Compile_Spec(spec: mujoco.MjSpec) -> mujoco.MjModel:
    """
    Description:
        Compile a spec (spec.compile(), the same model), with the one known warning (CONST_KNOWN_ATTACH_WARNING) replaced by
        the line CONST_KNOWN_ATTACH_NOTE on stderr. MuJoCo prints it on stderr when the robot is attached (Build_Cell_Spec
        removes that print) and raises it as a UserWarning here. Any other warning and any other stderr output are passed on.

    Args:
        (1) spec [mujoco.MjSpec]: The spec.

    Returns:
        (1) parameter [mujoco.MjModel]: The compiled model.
    """

    with _Known_Warning_Filter_Cls() as known:
        model = spec.compile()
    if known.Seen:
        sys.stderr.write(CONST_KNOWN_ATTACH_NOTE + '\n')
        sys.stderr.flush()
    return model


def Export_Cell_Scene(cell: Cell_Configuration_Str, path: str = CONST_CELL_SCENE_XML) -> str:
    """
    Description:
        Write the assembled cell as one MJCF for MuJoCo's viewer, with the mesh paths relative to the
        file's folder, so it loads from any working directory.

    Args:
        (1) cell [Cell_Configuration_Str(object)]: The cell configuration.
        (2) path [string]: The file (default models/cell_scene.xml).

    Returns:
        (1) parameter [string]: The absolute path written.
    """

    path = os.path.abspath(path)
    sc = Build_Cell_Spec(cell)
    Compile_Spec(sc)
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
