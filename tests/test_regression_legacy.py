"""Full physics cycle of the legacy plan (legacy/cell_plan.npz) against the brief §5.6 acceptance criteria.

The values come from the dict returned by the legacy run, not from the printed report. The limits are written here
on purpose (not imported from the CLI), so a changed limit in the code does not change the test.
"""
import pytest
from kinali_sim import cli

pytestmark = pytest.mark.slow


@pytest.fixture(scope='module')
def result(tmp_path_factory):
    import os
    cwd = os.getcwd(); os.chdir(tmp_path_factory.mktemp('run'))
    try:
        return cli.load_legacy_cycle().run()        # raises RuntimeError if MuJoCo resets the state (instability)
    finally:
        os.chdir(cwd)


def test_no_instability(result):
    assert result is not None
    assert abs(result['cycle_time'] - 65.8) < 0.05


@pytest.mark.parametrize('check', ['robot <-> MG06 / workstation / crates', 'robot tool <-> handle (outside closing / opening)',
                                   'robot <-> loader', 'loader <-> MG06 / workstation', 'PCB <-> lid / press plate'])
def test_no_collision(result, check):
    assert result['contacts'][check] is None, result['contacts'][check]


def test_four_cups_at_both_picks(result):
    picks = [e for e in result['vacuum'] if e['on']]
    assert [e['cups'] for e in picks] == [4, 4]


def test_pcb_in_mg06(result):
    assert result['placed']['error_mm'] < 0.5
    assert result['placed']['tilt_deg'] < 0.2


def test_pcb_in_output_crate(result):
    assert result['final_pcb']['dist_xy_mm'] < 2.0


def test_mg06_ends_open(result):
    assert abs(result['mg06_end']['lid_deg'] - (-39.96)) < 2.0
    assert abs(result['mg06_end']['handle_deg']) < 2.0
