"""`kinali-sim run` exit codes and acceptance logic, without running the physics."""
import copy
import pytest
from kinali_sim import cli

GOOD = {'contacts': {k: None for k in ['robot <-> MG06 / workstation / crates', 'robot <-> loader']},
        'vacuum': [{'t': 3.28, 'on': True, 'cups': 4}, {'t': 9.52, 'on': False, 'cups': None},
                   {'t': 54.76, 'on': True, 'cups': 4}, {'t': 62.44, 'on': False, 'cups': None}],
        'placed': {'error_mm': 0.30, 'tilt_deg': 0.02}, 'final_pcb': {'dist_xy_mm': 0.83},
        'mg06_end': {'lid_deg': -40.0, 'handle_deg': -0.0}}


def failing(res):
    return [name for name, ok, _ in cli.acceptance(res) if not ok]


def test_good_result_passes():
    assert failing(GOOD) == []


@pytest.mark.parametrize('path, value', [
    (('contacts', 'robot <-> loader'), {'depth_mm': -0.5, 'geoms': ('a', 'b'), 't': 1.0}),
    (('vacuum', 2, 'cups'), 3),
    (('placed', 'error_mm'), 0.5),
    (('placed', 'tilt_deg'), 0.2),
    (('final_pcb', 'dist_xy_mm'), 2.0),
    (('mg06_end', 'lid_deg'), -0.5),
    (('mg06_end', 'handle_deg'), 28.7),
])
def test_each_violation_fails(path, value):
    res = copy.deepcopy(GOOD); node = res
    for k in path[:-1]: node = node[k]
    node[path[-1]] = value
    assert len(failing(res)) == 1


class _Unstable:
    def run(self, plan=None):
        raise RuntimeError('simulation unstable at t=1.000 s () - MuJoCo reset the state')


class _Failing:
    def run(self, plan=None):
        r = copy.deepcopy(GOOD); r['placed']['error_mm'] = 1.0; return r


@pytest.mark.parametrize('cycle, code', [(_Unstable(), 2), (_Failing(), 1)])
def test_run_exit_code(monkeypatch, tmp_path, cycle, code):
    monkeypatch.setattr(cli, 'load_legacy_cycle', lambda: cycle)
    assert cli.main(['run', '--report', str(tmp_path / 'report.txt')]) == code
    assert (tmp_path / 'report.txt').is_file()
