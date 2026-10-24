# MJCF - the scene, the vendored robot, what is never changed

The cell is built by `src/Scene/Core.py` through `mujoco.MjSpec` from three files (SPEC §6, §7.2).
✅ = enforced by a test; 📄 = documentation only.

## The files

| File | Origin | Rule |
|---|---|---|
| `models/scene.xml` | built from Blender in `mg06_loader_mujoco` (workstation, MG06, loader, free PCB) | finished; **never regenerated**; edited by hand only for a decided change (the PCB colour, a mesh path) |
| `models/aubo_i5/aubo_i5.xml` + meshes | vendored from github.com/ian-chuang/Manipulator-Mujoco (MIT, `LICENSE_Manipulator-Mujoco`) | never edited; a patch is applied in code |
| `models/robot_tool/vgc10_opener_free.xml` + meshes | the VGC10 + fork of the MG06 project (a test variant with a free joint) | never edited; the free joint is removed in code |
| `models/cell_scene.xml` | written by `Example/Scene/export_cell_scene.py` | generated; committed for the viewer; never hand-edited |

Mesh paths: each MJCF's `meshdir` is relative to its own file; `cell_scene.xml` gets the relative path
from its folder to the meshes. ✅ `test/Scene/test_core.py` (every mesh file exists, from another working
directory).

## Patches applied in code 📄

`Scene.Core` (SPEC §7.2): the crates' filled hulls replaced by floor + 4 walls; the tool's joints removed
and the tool mounted rigidly on the flange; the cup block shortened 2 mm; one body, bellows slide joint,
mass sphere and square pad per cup; one adhesion actuator per cup; contact bits of robot and tool;
three self-collision exclusions; robot joint armature, damping and position servos; loader drive lag;
the look (lights, materials, cameras).

## Never changed in a structure or style change ✅

Timestep, solver options, any solref / solimp / solmix / margin / gap, friction, masses, geometry,
actuator gains and ranges, the vacuum / adhesion setup, the bellows springs, the CAD deviations
(design-decisions). Pinned by the fingerprint: `test/Scene/test_core.py` compares the compiled model with
`test/Scene/data/baseline_model_fingerprint.json`; the only accepted difference is
`geoms/pcb_vis/geom_rgba`.

## The integrator warning 📄

`aubo_i5.xml` sets `integrator="implicitfast"`; `scene.xml` sets none. MuJoCo keeps the parent's Euler
and warns at the attach (`KNOWN_ISSUES.md`). The verified physics ran with Euler; the warning is left as
it is.
