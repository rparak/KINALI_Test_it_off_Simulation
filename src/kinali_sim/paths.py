"""Folders and files of the repository, resolved from the package location (never from the working directory).

The package is used from the source tree (`pip install -e .`): models, assets and data live in the repository, not in
the installed package.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]           # src/kinali_sim/paths.py -> repository root
MODELS = ROOT / 'models'
ASSETS = ROOT / 'assets'
DATA = ROOT / 'data'
CONFIG = ROOT / 'config'
OUTPUT = ROOT / 'output'
LEGACY = ROOT / 'legacy'

SCENE_XML = MODELS / 'scene.xml'                     # finished scene (workstation, MG06, loader, PCB) - do not regenerate
CELL_SCENE_XML = MODELS / 'cell_scene.xml'           # whole cell incl. robot, written by `kinali-sim export-xml`
ROBOT_XML = ASSETS / 'robot' / 'aubo_i5' / 'aubo_i5.xml'
TOOL_XML = ASSETS / 'robot_tool' / 'vgc10_opener_free.xml'
PCB_META = DATA / 'pcb_meta.json'
FORK_PATH = DATA / 'fork_close_open_path.npz'
LEGACY_PLAN = LEGACY / 'cell_plan.npz'
REPORT = OUTPUT / 'report.txt'
