# Design decisions

Deliberate choices whose reason is not visible in the code. Each one says what would reopen it.

## The project follows project-style-rpx and the UR_PathGen_SF_1 layout, not the brief's package layout

**Decision.** The code, tests, scripts, installer and records follow the personal skills
(project-style-rpx first) with github.com/rparak/UR_PathGen_SF_1 as the structural template:
`src/<Domain>/Core.py` without `__init__.py`, `Example/` scripts, `test/` mirroring `src/`,
`install.sh` + `requirements.txt`, `.claude/project-docs/`. This replaces the layout of the brief §5.1
(package `kinali_sim`, `pyproject.toml`, console script `kinali-sim`, `tests/`, `config/`) wherever they
conflict (owner, 2026-10-24). Where the skills and the template disagree, the skills win and the
difference is recorded (`FEATURES.md` F3).

**Why.** One convention across the owner's projects; the skills carry the checker, the test discipline
and the records that keep a verified simulation verified.

**Alternatives.** The brief's package layout (step 1 was first done that way, commit `42a6121`):
rejected by the owner for consistency. Copying the template's code: rejected - only its structure and
conventions are taken.

**Revisit if** the owner's skills change their layout rules.

Pinned by `test/Docs/test_layout.py` (root entries, project docs, skill references).

## Simulation now, real-cell transfer planned

**Decision.** The project is a simulation today; keypoints and programs are intended to be transferred to
the real AUBO i5 later (owner, 2026-10-24). So: the formats stay controller-friendly (joint values in a
defined unit and order, the TCP definition, speeds as % of the datasheet, SPEC §17); the simulation-only
deviations (the CAD shifts in `scene.xml`, the assumed acceleration limits, the vacuum's adhesion model)
are checked against the real cell before a transfer; every output is marked as not validated for the
real cell (`KNOWN_ISSUES.md` opening, the README alert) until the real-cell validation procedure exists.

**Why.** A test on the real cell is planned; a format that cannot be loaded into the controller, or a
model deviation nobody re-checks, would make the transfer unsafe or redo the work.

**Alternatives.** Simulation only, formats free: rejected, the transfer is planned. Validating now:
not possible, the procedure and the real cell's measurements do not exist yet.

**Revisit if** the real-cell validation procedure is defined, or the transfer is cancelled.

Pinned by: nothing yet (the formats come in steps 3–4; their tests will pin unit and order).

## The MIT copyright holder is JIC Smart Factory; the author is Roman Parak

**Decision.** `LICENSE` and every file header read `Copyright (c) 2026 JIC Smart Factory` (the wording of the
template UR_PathGen_SF_1); the headers' `Author   :` line stays `Roman Parak`. The style checker enforces the
holder through `.claude/skills/project-style-rpx/references/check_style.json`. Vendored licences keep their own
holders (`models/aubo_i5/LICENSE_Manipulator-Mujoco`: ian-chuang/Manipulator-Mujoco, MIT). *Corrected
2026-10-24 (owner): until then the holder was Roman Parak, taken from the first `LICENSE`.*

**Why.** The project is JIC Smart Factory's internal research (SPEC §1); the copyright is the institution's,
the authorship the person's.

**Alternatives.** Roman Parak as the holder: corrected by the owner.

**Revisit if** the project's ownership changes.

Pinned by the style checker (`holder`), run before every commit.

## Brand: JIC Smart Factory, collaboration JIC Smart Factory × KINALI, product Test-it-off

**Decision.** An internal research project of JIC Smart Factory, part of its internal research supporting small and medium-sized enterprises in advanced simulation. Developed in collaboration with KINALI for the development of the Test-it-off system, an automated workcell for testing assembled printed circuit boards (PCBA). (owner, 2026-10-24). The project carries the JIC Smart Factory brand with the
KINALI logo; the product Test-it-off is referenced by name only (no logo):
- where: the README header (now) and the Phase 2 application's header (later);
- how: black SVG on light grounds, white SVG on dark grounds (GitHub dark mode, a possibly dark Phase 2 app),
  `Media/<Logo>_Black.svg` / `_White.svg`, written by `Example/Media/make_logo_variants.py` by recolouring the
  vector originals (paths and viewBox kept);
- the originals stay in `Media/Original/` for regeneration; the JIC Smart Factory original is byte-identical
  to jic-sf-ui-style's `assets/logos/jic-smart-factory.svg` (sha256 pinned in the test), the brand's source
  of truth;
- the JIC recolouring is a **temporary** project departure from jic-sf-ui-style ("never recoloured"),
  accepted by the owner (2026-10-24); an official monochrome JIC file has been requested from JIC and
  replaces the recoloured one when it arrives (`KNOWN_ISSUES.md`);
- the KINALI logo is used with KINALI's consent (owner confirmed, 2026-10-24);
- no funding band: the project has no external or public funding.

*Corrected 2026-10-24: the Test-it-off logo is not used (owner); its raster original was removed, which
resolves the former known issue "the Test-it-off logo exists only as a raster" (no tracing tool added).*

**Why.** One monochrome family reads on both grounds and beside each other; the owner's decision.

**Alternatives.** The JIC purple original as jic-sf-ui-style prescribes ("the purple version is for white
grounds"): overruled by the owner until the official monochrome file arrives. Tracing the raster
Test-it-off logo: dropped, the logo is not used.

**Revisit if** JIC Smart Factory supplies the official monochrome vector, KINALI supplies one, or the brand
rules change.

Pinned by `test/Example/Media/test_make_logo_variants.py` (originals' hashes, one colour, geometry kept,
byte-identical regeneration).

## The project is named "Test-it-off Simulation"

**Decision.** The project and application name is **Test-it-off Simulation**; the full title
**Test-it-off Simulation – Automated PCBA Testing Cell in MuJoCo**. It is used in the README title, SPEC's title and §1, `CLAUDE.md`
and, later, as the Phase 2 window title. The official spelling of the product is **Test-it-off** (with
hyphens) everywhere except verbatim quotes of external sources and the URL testitoff.cz. The repository
name `KINALI_Test_it_off_Simulation` stays unchanged (owner, 2026-10-24).

**Why.** One name for the project in every place a reader meets it; KINALI's own spelling of the product.

**Alternatives.** The repository name as the project name: rejected, it is a technical identifier.

**Revisit if** KINALI changes the product's name or spelling.

Pinned by: none (text); checked by `git grep` for the old spellings when docs change.

## Phase 2 GUI toolkit: PyQt5 or PyQt6 - open

**Open, not decided.** Recorded so the choice is made with its consequences in view.

- **PyQt5 (recommended by the brand skill):** jic-sf-ui-style is a ready PyQt5 package (palette, fonts,
  logos, pixel icons, example app, tests), so the brand comes for free; but Qt 5.15 is the end of Qt 5 and
  will age; MuJoCo's OpenGL in a `QOpenGLWidget` works on both.
- **PyQt6 (brief §8):** Qt 6 is the current line with long support; but the brand package would have to be
  ported (enum and API changes) and then maintained separately from the skill's PyQt5 version.

**Decide when** Phase 2 starts.

## The keypoint check measures exact distances, not generated contacts

**Decision.** The keypoint clearance (and, from F8, the program's segment check) is measured with `mj_geomDistance`
for every collidable pair within the clearance (`Collision.Close_Pairs`), not read from the contacts MuJoCo generates in
a model with an enlarged geom margin (the legacy planner's planning model). `Close_Pairs` filters the pairs with a
vectorised bounding-sphere broad phase (numpy, a 1e-9 m slack so it keeps every pair the per-pair test kept), then
measures each candidate exactly: the same pairs and distances as the per-pair loop, 38 times faster (owner's B2, F8.1).

**Why.** Generated contacts miss near pairs depending on unrelated geometry: the carried PCB 1.02 mm above the crate
floor at `output_place` produces no contact in the cell with the 6 mm margin, and 16 contacts once one more geom - even a
box 5 m away - is added to the workstation body (measured 2026-10-24, F8.1). *Corrected 2026-10-24: this entry first
blamed MuJoCo's box-box collider ignoring the margin; adding the table boxes showed the missing contact comes from the
workstation body's mid-phase, so contact generation is not a reliable distance measure at all.* Exact distances also
found the cup block 2.1 mm above a held PCB (now in `cups_pcb`) and the robot's mount (excluded by a code constant).
B2 (2026-10-24, Apple M2 Pro): all 22 keypoints and 500 random samples of `Data/Reference/cell_plan.npz` (seed
20261024, the PCB held by the cups), robot and PCB queries, cuts 3 and 50 mm - identical pair lists; 15.98 -> 0.42 ms
per sample at 3 mm (37.9x), 16.13 -> 0.56 ms at 50 mm (28.8x).

**Alternatives.** Contacts with a larger margin: rejected, whether a pair is generated depends on the model's other
geoms. Converting the boxes to meshes: a model change. The per-pair loop: 14-16 ms per configuration, too slow for a
4 ms trajectory.

**Revisit if** MuJoCo's contact generation becomes independent of unrelated geoms (then a margin model could be used
for speed), or `Close_Pairs` is still too slow for a check.

Pinned by `test/Collision/test_core.py::test_exact_distances_find_a_near_box_pair_whatever_contacts_are_generated`,
`::test_the_vectorised_broad_phase_gives_the_brute_force_pairs`, `test/Keypoints/test_core.py`.

## The check model is the planning model with the table top as two boxes

**Decision.** The table top, which `models/scene.xml` has only as visual meshes, is two boxes in
`Collision.Build_Planning_Model` only (`CONST_TABLE_BOXES`: the bounding boxes of the two plates of the visual mesh
`ws_vis_static_217_214_219`, no margin, the crates' collision bits 2 / 1 so they meet the robot and the PCB). The keypoint
check and the program check use the planning model (owner, 2026-10-24); the physics model and its fingerprint have no
table. The PCB against the table counts as `pcb_crate` only while the PCB is inside a crate (`Scene.Crate_Holding`: within
a crate's inner footprint, below its 149 mm walls - the crate floor lies between them); a carried PCB or a PCB in the jaws
keeps the 3 mm clearance to the table (owner, 2026-10-24).

**Why.** The robot and the carried PCB can reach the table; nothing modelled it. Measured: the robot stays at least 8.9 mm
from the boxes over the reference plan; the keypoint table is byte-identical with them; the PCB on a crate floor is
2.00 mm from the table (the floor's thickness) and the carried PCB at `output_place` 3.02 mm - a margin on the boxes'
top face would fail both, so none is added (a plate is a box).

**Alternatives.** The table in `scene.xml` or the physics model: rejected, the verified physics and its fingerprint stay
unchanged. A separate check model beside the planning model: two models that can disagree. The PCB-table pair always in
`pcb_crate`: rejected by the owner (it would hide a carried PCB scraping the table).

**Revisit if** the table is added to the physics model, or the CAD's table changes.

Pinned by `test/Collision/test_core.py::test_the_table_boxes_are_the_plates_of_the_table_mesh`,
`::test_the_physics_model_has_no_table_collision`, `::test_the_table_is_close_only_to_a_pcb_on_a_crate_floor`,
`test/Keypoints/test_core.py::test_the_contact_groups_of_a_pair`, `::test_crate_holding_tells_the_inside_of_a_crate`.

## A fork keypoint carries the MG06 state of the recorded path

**Decision.** `Fixture` is required for every keypoint: `Open`, `Closed` or the recorded `{Lid, Handle,
Source}` of `Data/Paths/fork_close_open_path.npz`; the press plate follows the handle through the model's
equality. `fork_closed_end` is row 286 (handle 29.13°), `fork_open_join` row 663, `fork_start` row 23; the
leave / return keypoints are settled `Closed` (owner, 2026-10-24, condition C1).

**Why.** The fork keypoints touch the handle bar, and the contact depends on the handle angle: at
`fork_closed_end` the bar is 0.33 mm into the fork in the recorded state and 5.95 mm in the settled closed
state (C1, FEATURES.md F6). Posing or checking in the wrong state reports a collision that is not there, or
hides one.

**Alternatives.** One MG06 state per program step: rejected, the state changes along the fork path.

**Revisit if** the fork path is re-recorded or replaced by MoveL keypoints (step 4).

Pinned by `test/Keypoints/test_core.py::test_the_fork_keypoints_in_their_recorded_state_touch_the_bar_at_most_half_a_millimetre`,
`::test_fork_closed_end_in_the_wrong_mg06_state_is_reported`.

## The contact groups are only those a keypoint or a program step observes

**Decision.** The check knows four contact groups, `cups_pcb`, `handle_bar`, `pcb_crate` and `pcb_jaws` (the PCB on the
loader's jaws), each with its deepest allowed overlap in `Configuration/Cell.yaml` `Clearance.Allowed_Depth` (1.6, 0.5,
0.5, 0.5 mm); a deeper overlap fails even where the group is allowed. A keypoint declares groups in `Allow_Contact`, a
program step in its `Allow_Contact` (owner, 2026-10-24); every declared group must be observed. *Corrected 2026-10-24:
`pcb_jaws` was removed in F6.4 because no keypoint observed it; it is back (owner, step 4 answer 6) because the handover and
take steps of the default program have the carried PCB between the closing jaws (F8).*

**Why.** A group nothing declares is a field that cannot take effect; allowing only what is present keeps an unexpected
contact visible.

**Alternatives.** Keeping `pcb_jaws` through F6: rejected then, nothing observed it until step 4.

**Revisit if** a group is no longer observed by any keypoint or step.

Pinned by `test/Configuration/test_configuration_files.py::test_an_invalid_keypoint_value_is_named`,
`::test_the_shipped_cell_configuration_loads_in_si_units`, `test/Keypoints/test_core.py::test_every_declared_contact_is_really_present`,
`::test_the_contact_groups_of_a_pair`.

## MuJoCo is pinned to 3.16.0

**Decision.** `mujoco==3.16.0` in `requirements.txt`; the installer checks the version.

**Why.** The physics was tuned and verified on 3.16 (SPEC §4.7): in 3.16 `gap` is not applied to
`includemargin`, which is why the PCB and the cups carry no `margin`/`gap`; another version may treat
contacts differently. A pinned version also keeps the baseline deterministic: on one machine and one
version the report is reproduced byte for byte (measured 2026-10-24, three runs, empty diffs).

**Alternatives.** An unpinned or `>=` dependency: rejected, a silent MuJoCo update could change every
contact number and the baseline would no longer be comparable.

**Revisit if** a MuJoCo release is needed for a feature; the upgrade is its own change with a new
baseline, the fingerprint re-measured and the margin behaviour re-checked.

Pinned by `test/Scene/test_core.py` (fingerprint), `test/Simulation/test_core.py` (slow regression) and `test/Install/test_install.py` (the version check).

## Python 3.11, pinned requirements, installed by install.sh

**Decision.** The interpreter is Python 3.11 (`/opt/homebrew/bin/python3.11` on macOS, `python3.11` on
Linux); every direct dependency is pinned `==` in `requirements.txt` with the owner's pin list; the
environment is created by `install.sh` into `.venv`. `pyproject.toml` is removed (owner, 2026-10-24).

**Why.** The same versions as the owner's other MuJoCo project, so both share one verified toolchain;
project-style-rpx prescribes the installer and pinned direct dependencies. The interpreter is named
explicitly because a bare `python3` on this machine is miniforge's.

**Alternatives.** Keeping the Python 3.14.7 / numpy 2.5.4 environment the baseline was first measured
in: rejected by the owner. The baseline was re-measured in the pinned environment (2026-10-24, Python
3.11.2, numpy 2.4.6, MuJoCo 3.16.0, `install.sh --recreate`): the cycle report is byte-identical to the
3.14 one and the fingerprint compare shows only `geoms/pcb_vis/geom_rgba`, so no new baseline was
needed and the 3.11 environment is the baseline environment (SPEC §4.6). `pyproject.toml` with an editable install: rejected, the skills use no package and
no `__init__.py`.

**Revisit if** a pinned package has no wheel for a target, or the owner moves both projects to a newer
Python.

Pinned by `test/Install/test_install.py` (16 tests, sourced with stubs).

## The legacy plan stays the regression input until step 6; the legacy planner is kept as a git tag

**Decision.** `Data/Reference/cell_plan.npz` (the verified IK plan) stays the regression input of the cycle run
(`Example/Simulation/run_cycle.py`, `test/Simulation`) until the keypoint program reproduces it (brief §7 step 6).
The legacy planner `legacy/plan_cell.py`, the legacy cycle test `legacy/test_cell_cycle.py` and their shims were
removed (owner, 2026-10-24, F4) after the annotated tag **`legacy-planner-reference`** (commit `f754e2d`) was set
on the last commit holding them. *Corrected 2026-10-24: until F4 the planner and the cycle test stayed in
`legacy/`, frozen.*

**Why.** The plan is the only verified end-to-end cycle, so every change before step 6 is measured against
it. The planner is not product code; step 4 needs it only as a reference for the fork-path joins, the
leave / return path around the closed handle and the keypoints, which `git show
legacy-planner-reference:legacy/plan_cell.py` gives. The port of the loop into `src/Simulation` was proven
before the removal: the same report and every measured value the same float as the legacy run (F3.5); the
planner regenerated the plan through the ported Kinematics and Collision code to 1.24e-14 rad (F3.4).

**Alternatives.** Keeping `legacy/` until step 6: rejected by the owner (root layout). Deleting the plan
too: rejected, nothing else would show that the physics is unchanged.

**Revisit if** step 6 passes: the keypoint program's trajectory replaces the plan as the regression input.

Pinned by `test/Simulation/test_core.py` (slow: the report equals the baseline; raw-value pins).

## The MG06 lock has 1.5 mm radial play

**Decision.** The lock's hook and pin in `models/scene.xml` have 1.5 mm radial play instead of the 0.5 mm of
`mg06_loader_mujoco` (the `GAP` of the MG06 build there).

**Why.** The robot pushes the lid down harder than the free-guided fork of the earlier version: with 0.5 mm
the lid sank 0.6 mm into the base, the hook hit the pin already at 27° and the handle never reached 29°.
With 1.5 mm the handle closes to 28.7° (SPEC §4.6) and the lid rises 0.5° in the play.

**Alternatives.** A softer lid contact: rejected, it would change the verified contact parameters.

**Revisit if** the real lock is measured.

Pinned by the fingerprint (`test/Scene/data/baseline_model_fingerprint.json`) and the report's
"MG06 closed" line (`test/Simulation/test_core.py`).

## Logs go to logger/, generated artefacts to Data/Generated/<Kind>/

**Decision.** One place per kind of file a run produces: logs (`logger/<script>.log`, MuJoCo's
`logger/MUJOCO_LOG.TXT`) in `logger/`; generated artefacts in `Data/Generated/<Kind>/` - the cycle report in
`Reports/`, proof images (`pcb_green.png`, later `logo_check.png`) in `Images/`, the step 4 trajectory in
`Trajectories/`. Both folders are git-ignored and created by `install.sh` and by the scripts; a script's
`--out DIR` moves its file. `models/cell_scene.xml` stays in `models/` (the viewer command loads it). The root
output folder is gone (owner, 2026-10-24, F4.4).

**Why.** The root holds only declared entries (style skill); `Data/Generated/` is the template's place for
generated output beside the inputs; logs are not results and keep their own folder. One folder per kind
keeps a report, an image and a trajectory from overwriting or hiding each other.

**Alternatives.** Reports and images in `logger/`: rejected, they are results a reader opens, not run logs.
Keeping an output folder at the root: rejected by the owner.

**Revisit if** step 4 makes the trajectory a committed deliverable: then `Data/Generated/Trajectories/` (or the
delivered file) is un-ignored, as the template commits its `Data/Generated/`.

Pinned by `test/Install/test_install.py::test_the_run_folders_are_created` and the `test/Example` script
tests (defaults and `--out`).

## The PCB is drawn in solder-mask green

**Decision.** The visual geom `pcb_vis` has rgba `0.05 0.42 0.18 1` in `models/scene.xml`; no material
(the look code skips `pcb*` geoms). The PCB's collision geoms keep their colours (group 3, hidden by
default).

**Why.** Brief goal 5: a realistic solder-mask green instead of orange. Measured on a `cam_pcb` render:
mean RGB 18/122/55 over the PCB's pixels (2026-10-24). The fingerprint shows exactly this one difference
from the baseline.

**Alternatives.** A material with a texture: not needed for the goal. Splitting the mesh into board and
components with their own colours: an open follow-up (`KNOWN_ISSUES.md`).

**Revisit if** the PCB mesh is split.

Pinned by `test/Scene/test_core.py::test_the_pcb_is_green_without_a_material` and `test/Render/test_core.py::test_the_pcb_camera_shows_a_green_pcb`.

## The studio look: fog on, a 1 m grid, a statistic from the cell's bounding box

**Decision.** The scene has the studio look of F7 (SPEC §4.7 Look), applied in code (`Scene.Core._Apply_Look`,
`CONST_LOOK_*`; `scene.xml` unchanged): a gradient sky, a dark 1 m grid on the existing visual-only floor plane
(20 m), a key spot light with shadows and a weak fill, haze and fog in the horizon colour, and an explicit
`statistic`. Fog is on wherever the project renders (offscreen images, the passive viewer; owner, 2026-10-24,
option A). Values that differ from the owner's first specification, each measured in MuJoCo 3.16.0
(2026-10-24):

- `texrepeat` 2 2, not 1 1: with `texuniform` MuJoCo maps one texture onto 2 m of a plane (orthographic top
  view: lines at −4, −2, 0, 2, 4 m); 2 2 gives a line every 1 m on the world axes.
- Sky `rgb2` 0.64 0.68 0.73, not the horizon colour: the gradient runs from `rgb1` at the zenith to `rgb2` at
  the nadir and the horizon is their mean, so `rgb2 = 2 × horizon − zenith` puts 0.42 0.45 0.49 at the horizon.
- `statistic` extent 2.6 m: `fogstart` / `fogend` are multiples of the extent, not metres. Without a
  statistic the 20 m plane moved the automatic extent to 4.4 m (fog 6.6–26 m, past the plane: a hard horizon
  line). The centre and extent come from the cell's bounding box at the default qpos, robot included, so the
  default free camera holds the whole cell with a 10 % margin (fog 3.9–15.6 m, inside the plane).
- Reflectance 0.05 (owner): 0.1 mirrored the cabinet clearly on the dark floor.
- The key light's shadows are hard: MuJoCo's shadow maps have no penumbra (the owner's "soft" key light is not
  available; edges look the same at cutoff 30° and 75°; no acne seen).

**Why.** With fog on the floor fades into the horizon colour with no edge: at the horizon the far floor renders
115.7 / 255 against the horizon's 115.6. The price: with fog on, MuJoCo draws the whole skybox in the fog colour,
so the sky is a uniform grey and the gradient shows only with fog off (the blocking viewer's default). Render time
is unchanged within noise (one `Render_Free_Image` call 223–256 ms after, 226–240 ms before; steady frame 25.5 →
23.2 ms): the shadow map was already 8192.

**Alternatives.** Option D (owner's prototype request): an infinite plane (size 0), fog off, haze on, the gradient
visible - rejected: a hard horizon line (floor 51 / 255 against the sky's 115 within 5 rows; the haze band is too
thin to hide it); its distant grid moiré was low. Look values in `Configuration/Cell.yaml`: rejected (owner), they
describe the presentation, not the cell. The spec's values as written: the 2 m grid, a darker horizon, the hard
horizon line above.

**Revisit if** MuJoCo stops fogging the skybox (then the gradient could show with fog on), adds soft shadows, or
the robot's default pose changes (re-derive the statistic).

Pinned by `test/Scene/test_core.py::test_the_floor_is_a_visual_plane_with_a_one_metre_grid`,
`::test_the_sky_is_the_gradient_with_the_horizon_colour_between`, `::test_haze_and_fog_are_the_horizon_colour`,
`::test_the_lights_are_one_key_spot_with_shadows_and_one_fill`, `::test_the_default_view_fits_the_whole_cell_with_a_margin`,
`::test_the_pcb_and_tool_cameras_are_unchanged`, `::test_the_crates_and_every_non_visual_field_equal_the_baseline`,
`test/Render/test_core.py::test_the_floor_grid_lines_are_one_metre_apart_on_whole_metres`,
`::test_the_offscreen_floor_fades_into_the_horizon_colour`, `::test_the_passive_viewer_switches_on_fog_and_haze`.

## Step 4 has two programs: the default one is taught, the optimized one is planned

**Decision.** Step 4 produces two versions of the same process (owner, 2026-10-24). The **default program**
(`Configuration/Program_Default.yaml`, `FEATURES.md` F8) is written as a human operator would teach it: named
sequential steps, every motion ending at rest at its point, no blending, the robot and the loader never moving
at the same time, approach and retreat points as offsets from keypoints, vias where a person would put them. For
it the brief's rule stands: no planning algorithm, collision freedom is checked, not searched. The **optimized
program** (F9) keeps every process-critical segment of the default program unchanged (LOCKED) and replaces only
the free-space transfers between them (FREE) by planned motion - a joint-space search with a fixed seed, shortcut,
smoothing, time-optimal timing within the robot's limits - and lets the robot and the loader move in parallel
under interlocks. Both versions pass the same segment checks (step 4) and, later, the physics checks of steps 5
and 6. The brief's "no planning algorithm" (SPEC §2 item 4) is thereby narrowed to the default program; SPEC §2
and §3 say so.

**Why.** The default program is the readable, teachable reference a person can check line by line and transfer
to a teach pendant; the optimized program shows how much of the cycle time is free-space motion and waiting, on
the same process and the same checks, so the two can be compared step by step.

**Alternatives.** One program only, taught (the brief): no measure of the achievable cycle time. One program
only, optimized: nothing a person can read and check against the process; a planner bug would change the
process-critical moves. Optimizing everything (contact moves included): rejected, the contact moves are the
verified physics (SPEC §4.7) and stay as taught.

**Revisit if** the optimized program cannot pass the physics checks of steps 5 or 6, or the owner decides one
version is enough.

Pinned by: nothing yet (F8 and F9 are designed, not implemented); their tests are listed in `FEATURES.md` F8 and
F9.

## The CAD deviations stay compensated in scene.xml

**Decision.** `models/scene.xml` keeps three deliberate deviations from the CAD: the loader shifted
+100 mm in Y and +0.95 mm in X; the rear-right holder support (`holder_10`) lowered by 0.8 mm; the
electrical cabinet 200 mm deep instead of 300 mm. The scene is never regenerated from Blender.

**Why.** Without them the cycle does not work in the model (SPEC §4.10; the previous Czech README): the loader
lacks about 75 mm of stroke without the +100 mm; the jaw centre sits 0.95 mm left of the nest centre and
the nest has no width play, so the PCB lands on a pin edge and stays tilted 1.4°; `holder_10` is 0.8 mm
higher than the other three supports and the PCB's centre of mass lies outside the other three, so the
board always tilts onto it; with the 300 mm cabinet the Z-axis motor runs about 65 mm into it at
insertion.

**Alternatives.** Correcting the CAD: the owner's to decide (the holder may be a model error or a cut-out
in the PCB, recommended to check). Regenerating the scene: rejected, it would drop the corrections.

**Revisit if** the CAD is corrected or the real cell is measured.

Pinned by the fingerprint (`test/Scene/data/baseline_model_fingerprint.json`).
