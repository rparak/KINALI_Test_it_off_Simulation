"""Legacy shim: the names of the former cell.py, mapped onto src/ (Configuration.Core, Scene.Core), so that
legacy/plan_cell.py and legacy/test_cell_cycle.py keep their code unchanged until step 6 removes legacy/.
Importing it also routes MuJoCo's log into logger/ (a script-level side effect, as in Example/)."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))
from Configuration.Core import Read_PCB_Meta, Configure_MuJoCo_Log, CONST_FORK_PATH
from Scene import Core as _scene

Configure_MuJoCo_Log()
_meta = Read_PCB_Meta()
P = {'cad_pos': _meta.Cad_Pos, 'place_pos': _meta.Place_Pos, 'hold_pos': _meta.Hold_Pos, 'quat': _meta.Quat,
     'y_home': _meta.Y_Home, 'y_place': _meta.Y_Place, 'z_place': _meta.Z_Place, 'jaw_grip': _meta.Jaw_Grip,
     'jaw_open': _meta.Jaw_Open, 'machine_shift': _meta.Machine_Shift}
FORK_PATH = CONST_FORK_PATH

ROBOT_BASE, ROBOT_YAW, ROBOT_JOINTS = _scene.CONST_ROBOT_BASE, _scene.CONST_ROBOT_YAW, _scene.CONST_ROBOT_JOINTS
KP, KV, FMAX, ARMATURE, VMAX_DATASHEET = _scene.CONST_KP, _scene.CONST_KV, _scene.CONST_FMAX, _scene.CONST_ARMATURE, _scene.CONST_VMAX_DATASHEET
CUP_R, CUP_N, VACUUM_PA, VACUUM_FORCE = _scene.CONST_CUP_R, _scene.CONST_CUP_N, _scene.CONST_VACUUM_PA, _scene.CONST_VACUUM_FORCE
CUP_XY, LOADER_TAU = _scene.CONST_CUP_XY, _scene.CONST_LOADER_TAU
CRATES, CRATE_FLOOR, CRATE_TOP = _scene.CONST_CRATES, _scene.CONST_CRATE_FLOOR, _scene.CONST_CRATE_TOP
PCB_HALF_T, SUCTION_IN_PCB, R_TOOL_IN_PCB = _scene.CONST_PCB_HALF_T, _scene.CONST_SUCTION_IN_PCB, _scene.CONST_R_TOOL_IN_PCB
crate_center, pcb_in_crate = _scene.Crate_Center, _scene.PCB_In_Crate
build_spec, build_model, export_xml = _scene.Build_Cell_Spec, _scene.Build_Cell_Model, _scene.Export_Cell_Scene
