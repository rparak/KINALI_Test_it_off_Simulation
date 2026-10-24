import importlib.util
import os
import sys
# pyproject.toml is gone (FEATURES.md F3.1): reach the brief-layout package through src/ until F3.6.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
import pytest
import mujoco
from kinali_sim import paths


@pytest.fixture(scope='session')
def fingerprint_tool():
    """tools/model_fingerprint.py imported as a module (it is a script, not part of the package)"""
    spec = importlib.util.spec_from_file_location('model_fingerprint', paths.ROOT / 'tools' / 'model_fingerprint.py')
    mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='session')
def cell_model():
    from kinali_sim import model
    return model.build_model()


@pytest.fixture
def pcb_vis_id(cell_model):
    return mujoco.mj_name2id(cell_model, mujoco.mjtObj.mjOBJ_GEOM, 'pcb_vis')
