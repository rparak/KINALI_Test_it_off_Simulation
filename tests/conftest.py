import importlib.util
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
