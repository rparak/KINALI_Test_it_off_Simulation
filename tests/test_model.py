"""The compiled model is the verified one: its fingerprint matches docs/baseline_model_fingerprint.json."""
import json
from kinali_sim import paths


def test_build_model(cell_model):
    m = cell_model
    assert (m.nq, m.nv, m.nu, m.nbody, m.ngeom, m.nmesh, m.neq) == (24, 23, 13, 25, 509, 438, 2)
    assert m.opt.timestep == 0.0005


def test_fingerprint_matches_baseline(cell_model, fingerprint_tool):
    ref = json.loads((paths.ROOT / 'docs' / 'baseline_model_fingerprint.json').read_text())
    cur = json.loads(json.dumps(fingerprint_tool.fingerprint(cell_model)))
    diff = fingerprint_tool.compare(cur, ref)
    assert diff == []
