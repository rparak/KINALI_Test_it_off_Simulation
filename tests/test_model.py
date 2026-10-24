"""The compiled model is the verified one: its fingerprint matches docs/baseline_model_fingerprint.json, except the
colour of the PCB visual (green since brief §7 step 2)."""
import json
import numpy as np
from kinali_sim import paths

GREEN = [0.05, 0.42, 0.18, 1.0]                  # solder-mask green (brief §2 goal 5)


def test_build_model(cell_model):
    m = cell_model
    assert (m.nq, m.nv, m.nu, m.nbody, m.ngeom, m.nmesh, m.neq) == (24, 23, 13, 25, 509, 438, 2)
    assert m.opt.timestep == 0.0005


def test_fingerprint_matches_baseline_except_pcb_colour(cell_model, fingerprint_tool):
    ref = json.loads((paths.ROOT / 'docs' / 'baseline_model_fingerprint.json').read_text())
    cur = json.loads(json.dumps(fingerprint_tool.fingerprint(cell_model)))
    diff = fingerprint_tool.compare(cur, ref)
    assert [l.split(':')[0] for l in diff] == ['geoms/pcb_vis/geom_rgba']


def test_pcb_is_green(cell_model, pcb_vis_id):
    assert np.allclose(cell_model.geom_rgba[pcb_vis_id], GREEN)
    assert cell_model.geom_matid[pcb_vis_id] == -1               # no material that could override the colour


def test_pcb_collision_geoms_unchanged(cell_model):
    for n, rgba in [('pcb_board', [1, .5, 0, .4]), ('pcb_comp', [1, .5, 0, .2])]:
        g = cell_model.geom(n)
        assert g.group == 3 and np.allclose(g.rgba, rgba)
