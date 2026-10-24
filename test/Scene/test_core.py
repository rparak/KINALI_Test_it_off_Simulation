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
# File Name: ../test/Scene/test_core.py
## =========================================================================== ##

# System (Default)
import os
import sys
# Subprocess (Subprocess management)
import subprocess
# JSON (JSON encoder and decoder)
import json
# XML (ElementTree XML API)
import xml.etree.ElementTree as ET
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Custom Lib.: the module under test
#   ../Scene/Core
from Scene.Core import Crate_Center, PCB_In_Crate, Export_Cell_Scene, Scene_Error_Cls, _Load_Spec
#   ../Configuration/Core
from Configuration.Core import CONST_SCENE_XML, CONST_ROBOT_XML, CONST_TOOL_XML, CONST_CELL_SCENE_XML

"""
Description:
    Initialization of constants.

    The compiled cell is the verified one (SPEC §4.3, §10.2). Expected values are written out here from
    SPEC §4.3 / §4.4 and the green-PCB decision.
"""
# The solder-mask green of the PCB visual (design-decisions: the PCB is drawn in solder-mask green).
#   Unit [Vector<float> 1x4] (-)
CONST_GREEN = [0.05, 0.42, 0.18, 1.0]
# The one accepted difference from the baseline fingerprint (the green PCB).
#   Unit [List<string>]
CONST_ACCEPTED_DIFFERENCES = ['geoms/pcb_vis/geom_rgba']
# The studio look (F7, SPEC §4.7 Look): the sky's zenith, horizon and nadir colours.
#   Unit [List<float>] (rgb 0-1)
CONST_SKY_ZENITH, CONST_HORIZON, CONST_SKY_NADIR = [0.20, 0.22, 0.25], [0.42, 0.45, 0.49], [0.64, 0.68, 0.73]
# The cameras on the PCB and the tool, as before the studio look: name -> position (m), fovy (deg), target body.
#   Unit [Dict<string, Tuple>]
CONST_CAMERAS = {'cam_pcb': ([-0.55, -0.85, 0.75], 40.0, 'pcb'), 'cam_tool': ([-0.30, 0.35, 0.80], 45.0, 'aubo/tool/ee_tool')}
# The crate centres of SPEC §4.4 (x, y).
#   Unit [Dict<string, Vector<float> 1x2>] (m)
CONST_CRATE_CENTRES = {'crate_pick': [0.15720, -0.75160], 'crate_place': [0.40620, -0.75160]}


def _Mesh_Files(xml_path):
    """
    Description:
        The absolute paths of all <mesh file=...> of an MJCF, resolved as MuJoCo does (meshdir relative
        to the file).
    """

    root = ET.parse(xml_path).getroot()
    meshdir = next((c.get('meshdir') for c in root.iter('compiler') if c.get('meshdir')), '')
    base = os.path.normpath(os.path.join(os.path.dirname(xml_path), meshdir))
    return [os.path.join(base, m.get('file')) for m in root.iter('mesh') if m.get('file')]


def test_the_model_has_the_spec_sizes(cell_model):
    """
    Description:
        The compiled cell has the sizes and timestep of SPEC §4.3.
    """

    m = cell_model
    assert (m.nq, m.nv, m.nu, m.na, m.nbody, m.ngeom, m.nmesh, m.neq) == (24, 23, 13, 2, 25, 509, 438, 2)
    assert m.opt.timestep == 0.0005


def test_the_fingerprint_equals_the_baseline_except_the_green_pcb(root, cell_model, fingerprint_tool):
    """
    Description:
        Every hashed property of the compiled model equals test/Scene/data/baseline_model_fingerprint.json
        except the PCB visual's colour: the restructure changed nothing physical or visual.
    """

    with open(os.path.join(root, 'test', 'Scene', 'data', 'baseline_model_fingerprint.json'), encoding='utf-8') as handle:
        reference = json.load(handle)
    current = json.loads(json.dumps(fingerprint_tool.Model_Fingerprint(cell_model)))
    differences = fingerprint_tool.Compare_Fingerprints(current, reference)
    assert [line.split(':')[0] for line in differences] == CONST_ACCEPTED_DIFFERENCES


def test_the_crates_and_every_non_visual_field_equal_the_baseline(root, cell_model, fingerprint_tool):
    """
    Description:
        The studio look changes only visual fields: every crate geom (the floors and walls) equals its baseline entry,
        and every accepted difference is the floor plane's size, a floor-material field, the number of lights or the
        PCB colour.
    """

    with open(os.path.join(root, 'test', 'Scene', 'data', 'baseline_model_fingerprint.json'), encoding='utf-8') as handle:
        reference = json.load(handle)
    current = json.loads(json.dumps(fingerprint_tool.Model_Fingerprint(cell_model)))
    crates = [k for k in reference['geoms'] if k.startswith('crate_')]
    assert len(crates) == 10 and all(current['geoms'][k] == reference['geoms'][k] for k in crates)
    visual = ('geoms/floor/geom_size', 'geoms/pcb_vis/geom_rgba', 'materials/grid/mat_', 'sizes/nlight')
    assert all(d.startswith(visual) for d in CONST_ACCEPTED_DIFFERENCES)


def test_the_floor_is_a_visual_plane_with_a_one_metre_grid(cell_model):
    """
    Description:
        The floor plane takes part in no contact; its material is the grid texture (200 px, a line at the edge),
        uniform per metre and repeated twice per unit length (MuJoCo maps one texture onto 2 m: one line per 1 m),
        with a subtle reflection.
    """

    m = cell_model
    floor, material, texture = m.geom('floor'), m.material('grid'), m.texture('grid')
    assert floor.type == mujoco.mjtGeom.mjGEOM_PLANE and (floor.contype, floor.conaffinity) == (0, 0)
    assert floor.matid == material.id and list(floor.size) == [20, 20, 0.05]
    assert m.mat_texid[material.id][mujoco.mjtTextureRole.mjTEXROLE_RGB] == texture.id
    assert bool(m.mat_texuniform[material.id]) and list(material.texrepeat) == [2, 2]
    assert (m.tex_width[texture.id], m.tex_height[texture.id]) == (200, 200)
    assert material.reflectance == pytest.approx(0.05) and material.specular == pytest.approx(0.2)


def test_the_sky_is_the_gradient_with_the_horizon_colour_between(cell_model):
    """
    Description:
        The skybox is the gradient from the zenith to the nadir; the side faces' middle rows (the horizon) are the
        horizon colour, the mean of the two (measured 2026-10-24: MuJoCo's gradient runs zenith to nadir).
    """

    m = cell_model
    t = m.texture('sky')
    w, h, n = m.tex_width[t.id], m.tex_height[t.id], m.tex_nchannel[t.id]
    faces = m.tex_data[m.tex_adr[t.id]:m.tex_adr[t.id] + w * h * n].reshape(6, w, w, n)[..., :3] / 255.0
    assert t.type == mujoco.mjtTexture.mjTEXTURE_SKYBOX
    assert faces[2, w // 2, w // 2] == pytest.approx(CONST_SKY_ZENITH, abs=0.005)
    assert faces[3, w // 2, w // 2] == pytest.approx(CONST_SKY_NADIR, abs=0.005)
    for f in (0, 1, 4, 5):
        assert faces[f, w // 2, w // 2] == pytest.approx(CONST_HORIZON, abs=0.005), f


def test_haze_and_fog_are_the_horizon_colour(cell_model):
    """
    Description:
        Haze and fog are the horizon colour, so the floor fades into the sky without a band; fog from 1.5 to 6
        extents, haze 0.6.
    """

    v = cell_model.vis
    assert list(v.rgba.haze) == pytest.approx(CONST_HORIZON + [1.0]) and list(v.rgba.fog) == pytest.approx(CONST_HORIZON + [1.0])
    assert (v.map.fogstart, v.map.fogend, v.map.haze) == pytest.approx((1.5, 6.0, 0.6))


def test_the_lights_are_one_key_spot_with_shadows_and_one_fill(cell_model):
    """
    Description:
        Two lights: the key spot light above the cell (cutoff 75 deg) casting shadows and the directional fill without.
    """

    m = cell_model
    key, fill = m.light('key_light'), m.light('fill_light')
    assert m.nlight == 2 and key.type == mujoco.mjtLightType.mjLIGHT_SPOT and bool(key.castshadow)
    assert list(key.pos) == pytest.approx([1.6, 1.2, 4.0]) and key.cutoff == pytest.approx(75.0)
    assert fill.type == mujoco.mjtLightType.mjLIGHT_DIRECTIONAL and not bool(fill.castshadow)
    assert (m.vis.quality.shadowsize, m.vis.quality.offsamples) == (8192, 8)


def test_the_default_view_fits_the_whole_cell_with_a_margin(cell_model):
    """
    Description:
        The statistic fits the cell's bounding sphere at the default qpos, robot included, into the default free
        camera (fovy 45 deg) with a margin of at least 5 % (designed 10 %): the robot no longer touches the edge.
    """

    m = cell_model
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    points = []
    for g in range(m.ngeom):
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            k = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[k]:m.mesh_vertadr[k] + m.mesh_vertnum[k]]
            points.append(v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])
        elif m.geom_type[g] != mujoco.mjtGeom.mjGEOM_PLANE:
            points.append(d.geom_xpos[g] + m.geom_rbound[g] * np.array([[-1, -1, -1], [1, 1, 1]]))
    points = np.vstack(points)
    low, high = points.min(0), points.max(0)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(m, camera)
    assert np.allclose(camera.lookat, (low + high) / 2, atol=0.002)
    radius = np.linalg.norm(high - low) / 2
    assert camera.distance * np.sin(np.radians(m.vis.global_.fovy / 2)) >= 1.05 * radius


@pytest.mark.parametrize('name', sorted(CONST_CAMERAS))
def test_the_pcb_and_tool_cameras_are_unchanged(cell_model, name):
    """
    Description:
        cam_pcb and cam_tool keep their position, field of view and target (the statistic does not move them).
    """

    position, fovy, target = CONST_CAMERAS[name]
    c = cell_model.camera(name)
    assert list(c.pos) == pytest.approx(position) and c.fovy == pytest.approx(fovy)
    assert c.mode == mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY and c.targetbodyid == cell_model.body(target).id


def test_the_pcb_is_green_without_a_material(cell_model):
    """
    Description:
        pcb_vis has the solder-mask green and no material that could override it.
    """

    g = cell_model.geom('pcb_vis')
    assert np.allclose(g.rgba, CONST_GREEN) and g.matid == -1


def test_the_pcb_collision_geoms_keep_their_colours(cell_model):
    """
    Description:
        The PCB's collision boxes (group 3) keep their translucent orange.
    """

    for name, rgba in [('pcb_board', [1, .5, 0, .4]), ('pcb_comp', [1, .5, 0, .2])]:
        g = cell_model.geom(name)
        assert g.group == 3 and np.allclose(g.rgba, rgba), name


@pytest.mark.parametrize('xml_path', [CONST_SCENE_XML, CONST_ROBOT_XML, CONST_TOOL_XML, CONST_CELL_SCENE_XML])
def test_every_mesh_file_of_every_mjcf_exists(xml_path):
    """
    Description:
        Each MJCF names its meshes relative to its own folder, and every one exists.
    """

    files = _Mesh_Files(xml_path)
    missing = [f for f in files if not os.path.isfile(f)]
    assert files and not missing, '%d of %d missing, e.g. %s' % (len(missing), len(files), missing[:3])


def test_the_committed_cell_scene_loads_from_any_working_directory(root, tmp_path):
    """
    Description:
        models/cell_scene.xml loads in a fresh interpreter started elsewhere (what the viewer does).
    """

    code = 'import mujoco; m = mujoco.MjModel.from_xml_path(%r); print(m.nq, m.nu, m.nmesh)' % CONST_CELL_SCENE_XML
    result = subprocess.run([sys.executable, '-c', code], cwd=str(tmp_path), capture_output=True, text=True, timeout=300)
    assert result.returncode == 0, result.stderr
    assert result.stdout.split() == ['24', '13', '438']


def test_an_export_elsewhere_loads_and_equals_the_committed_one(tmp_path, cell_configuration):
    """
    Description:
        Export_Cell_Scene into another folder writes mesh paths relative to that folder, so it loads; apart
        from meshdir it is the committed file.
    """

    path = Export_Cell_Scene(cell_configuration, str(tmp_path / 'cell_scene.xml'))
    assert mujoco.MjModel.from_xml_path(path).nq == 24
    with open(path, encoding='utf-8') as handle:
        exported = ET.fromstring(handle.read())
    with open(CONST_CELL_SCENE_XML, encoding='utf-8') as handle:
        committed = ET.fromstring(handle.read())
    for tree in (exported, committed):
        for c in tree.iter('compiler'):
            c.set('meshdir', '')
    assert ET.tostring(exported) == ET.tostring(committed)


@pytest.mark.parametrize('name', sorted(CONST_CRATE_CENTRES))
def test_the_crates_have_the_spec_centres(name):
    """
    Description:
        The crate centres and the PCB lying flat on a crate floor (z = 0.004 + 0.0009) of SPEC §4.4.
    """

    assert Crate_Center(name) == pytest.approx(CONST_CRATE_CENTRES[name], abs=5e-6)
    position, quaternion = PCB_In_Crate(name)
    assert position == pytest.approx(CONST_CRATE_CENTRES[name] + [0.0049], abs=5e-6)
    assert list(quaternion) == [1.0, 0.0, 0.0, 0.0]


def test_a_model_file_that_does_not_load_is_named(tmp_path):
    """
    Description:
        A missing model file is a Scene_Error_Cls naming it.
    """

    with pytest.raises(Scene_Error_Cls, match='absent.xml does not load'):
        _Load_Spec(str(tmp_path / 'absent.xml'))
