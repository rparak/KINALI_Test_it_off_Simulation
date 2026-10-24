"""Every file the model and the legacy run need exists, and is found from any working directory."""
import os
import xml.etree.ElementTree as ET
from pathlib import Path
import pytest
import mujoco
from kinali_sim import paths


def mesh_files(xml_path):
    """absolute paths of all <mesh file=...> of an MJCF, resolved like MuJoCo does (meshdir relative to the file)"""
    xml_path = Path(xml_path); root = ET.parse(xml_path).getroot()
    meshdir = next((c.get('meshdir') for c in root.iter('compiler') if c.get('meshdir')), '')
    base = (xml_path.parent / meshdir).resolve()
    return [base / m.get('file') for m in root.iter('mesh') if m.get('file')]


@pytest.fixture(autouse=True)
def other_cwd(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)


def test_root_is_the_repository():
    assert (paths.ROOT / 'pyproject.toml').is_file()
    assert Path.cwd() != paths.ROOT


@pytest.mark.parametrize('f', ['SCENE_XML', 'ROBOT_XML', 'TOOL_XML', 'PCB_META', 'FORK_PATH', 'LEGACY_PLAN'])
def test_named_files_exist(f):
    assert getattr(paths, f).is_file(), getattr(paths, f)


@pytest.mark.parametrize('xml', ['SCENE_XML', 'ROBOT_XML', 'TOOL_XML', 'CELL_SCENE_XML'])
def test_mesh_files_exist(xml):
    files = mesh_files(getattr(paths, xml))
    assert files, f'no meshes found in {xml}'
    missing = [str(f) for f in files if not f.is_file()]
    assert not missing, f'{len(missing)} of {len(files)} meshes missing, e.g. {missing[:3]}'


def test_build_model_from_other_cwd():
    from kinali_sim import model
    m = model.build_model()
    assert (m.nq, m.nu) == (24, 13)


def test_exported_scene_loads_from_other_cwd():
    m = mujoco.MjModel.from_xml_path(os.fspath(paths.CELL_SCENE_XML))
    assert (m.nq, m.nu, m.nmesh) == (24, 13, 438)
