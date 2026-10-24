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
floor at `P_OUTPUT_PLACE` produces no contact in the cell with the 6 mm margin, and 16 contacts once one more geom - even a
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
2.00 mm from the table (the floor's thickness) and the carried PCB at `P_OUTPUT_PLACE` 3.02 mm - a margin on the boxes'
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
equality. `P_MG06_CLOSE_END` is row 286 (handle 29.13°), `P_MG06_OPEN_JOIN` row 663, `P_MG06_FORK_START` row 23; the
leave / return keypoints are settled `Closed` (owner, 2026-10-24, condition C1).

**Why.** The fork keypoints touch the handle bar, and the contact depends on the handle angle: at
`P_MG06_CLOSE_END` the bar is 0.33 mm into the fork in the recorded state and 5.95 mm in the settled closed
state (C1, FEATURES.md F6). Posing or checking in the wrong state reports a collision that is not there, or
hides one.

**Alternatives.** One MG06 state per program step: rejected, the state changes along the fork path.

**Revisit if** the fork path is re-recorded or replaced by MoveL keypoints (step 4).

Pinned by `test/Keypoints/test_core.py::test_the_fork_points_in_their_recorded_state_touch_the_bar_at_most_half_a_millimetre`,
`::test_the_close_end_in_the_wrong_mg06_state_is_reported`.

## The contact groups are only those a keypoint or a program step observes

**Decision.** The check knows four contact groups, `cups_pcb`, `handle_bar`, `pcb_crate` and `pcb_jaws` (the PCB on the
loader's jaws), each with its deepest allowed overlap in `Configuration/Cell.yaml` `Clearance.Allowed_Depth` (1.6, 0.6,
0.5, 0.5 mm); a deeper overlap fails even where the group is allowed. `handle_bar` is 0.6 mm (owner, 2026-10-24): the
recorded `PATH_FORK_CLOSE` path overlaps the bar by 0.539 mm at one row - the physics-verified path (up to 294 N on the bar,
SPEC §4.6) - measured plus a margin; it was 0.5 mm, and the default program's check failed on that one row (F8.5). A keypoint declares groups in `Allow_Contact`, a
program step in its `Allow_Contact` (owner, 2026-10-24); every declared group must be observed. *Corrected 2026-10-24:
`pcb_jaws` was removed in F6.4 because no keypoint observed it; it is back (owner, step 4 answer 6) because the handover and
take steps of the default program have the carried PCB between the closing jaws (F8).*

**Why.** A group nothing declares is a field that cannot take effect; allowing only what is present keeps an unexpected
contact visible.

**Alternatives.** Keeping `pcb_jaws` through F6: rejected then, nothing observed it until step 4.

**Revisit if** a group is no longer observed by any keypoint or step.

Pinned by `test/Configuration/test_program_parser.py::test_an_invalid_constant_is_named_with_its_file_and_line`,
`test/Configuration/test_configuration_files.py::test_the_shipped_cell_configuration_loads_in_si_units`, `test/Keypoints/test_core.py::test_every_declared_contact_is_really_present`,
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

**Revisit if** step 6 makes the trajectory the regression input: then the delivered file is un-ignored. *Corrected
2026-10-24 (owner, step 4 answer 11): the trajectories stay git-ignored until step 6; F9's optimized program goes to the
committed `Data/Optimized/`.*

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
(`Configuration/Program.yaml`, `FEATURES.md` F8, F10) is written as a human operator would teach it: named
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

## A program's robot points are stored joints; lines and paths end exactly on them

**Decision.** Every robot point of a program is a keypoint's stored `Joints` (F6's determinism rule). A MoveJ interpolates
joints between stored joints and its last sample is the target's stored joints exactly (since F9.6: `q0 + (q1 - q0) x 1.0`
differed from them by up to a rounding, so the optimized program's LOCKED parts after a MoveJ differed by 1.6e-15 rad); a MoveL aims at the pose of its target's stored joints and a `Path` at its recorded rows, with
IK per sample seeded from the previous sample, and their last sample is replaced by the target's stored joints when the IK
end is on the same branch (within the stale tolerance and 1e-3 rad per joint); otherwise the build fails, and a line that
would end in the other IK family fails with that reason. The per-sample IK is pure damped least squares (no null-space pull)
to 1e-8 m (`Program.CONST_IK_TOL`).

**Why.** The trajectory reproduces the keypoints exactly, so the program and the keypoint check pose the same robot and a
later step joins segments without a jump. 1e-8 m keeps the finite-difference speeds and accelerations free of IK noise
(1e-5 m between samples would read as about 2 rad/s²). The null-space pull to the seed held the solution about 5e-9 m off,
so the solver ran to its 200-iteration limit: the build took 33 s; without it 1.2 s, the same samples to the tolerance
(measured 2026-10-24). Whether a keypoint is stale against its `Reference` point moved by its `Offset` is the keypoint check's question, not the
build's (a line aims at the stored joints' pose).

**Alternatives.** Ending a line on its IK result: a sub-micrometre jump at the next MoveJ's start. Keypoint IK settings
(1e-5 m, the null-space pull): noisy speeds and a 33 s build.

**Revisit if** a keypoint is defined without stored joints, or the IK changes.

Pinned by `test/Program/test_core.py::test_movel_is_straight_and_ends_on_the_stored_joints`,
`::test_a_movel_cannot_switch_the_ik_family`, `::test_the_fork_paths_follow_the_recorded_rows_and_join_rule`.

## The carried PCB hangs at the cup face once the bellows extend

**Decision.** In a program's world state the PCB on the cups is the nominal held pose (the cup face, SPEC §4.4) set back
along the tool axis by the bellows' compression left: the compression when the vacuum went on (1.5 mm at a press, 0 at the
handover) less the TCP's travel since. The PCB therefore stays on its support for the first 1.5 mm of a lift and then hangs
at the cup face. *Corrected 2026-10-24 (F8.4): Part A froze the PCB's pose relative to the TCP at vacuum on; that kept the
1.5 mm compression for the whole carry, so the PCB reached `P_OUTPUT_PLACE` 2.5 mm above the crate floor instead of 1 mm, and
the place rule (vacuum off within 2 mm of the floor pose) failed.*

**Why.** Each cup sits on a sprung bellows (SPEC §4.10): under vacuum the board is pulled to the cup face as the bellows
extend. The keypoints assume the nominal relation (`P_OUTPUT_PLACE` = the PCB 1 mm above the floor).

**Alternatives.** The nominal pose from vacuum on: the PCB would jump 1.5 mm into the crate floor at the pick. The frozen
relative pose: above.

**Revisit if** step 5's physics shows the board hanging elsewhere in the cups (PCB slip is reported there).

Pinned by `test/Program/test_core.py::test_the_carried_pcb_hangs_at_the_cup_face_once_the_bellows_extend`,
`::test_the_pcb_moves_through_the_cycle_by_the_rules`.

## PATH_FORK_OPEN keeps the recorded velocity step at its join

**Decision.** `Robot.MovePath(PATH_FORK_OPEN)` starts at its join row as recorded: the flange already moves at 39.1 mm/s there, and the MoveL
onto `P_MG06_OPEN_JOIN` ends at rest, so the trajectory has a velocity step at the join (owner, 2026-10-24, answer 9). The build
notes it; step 5 reports the tracking error and the force on the bar there.

**Why.** The verified physics ran the reference plan with the same step (SPEC §4.6); a time warp would change the recorded
opening the regression was measured on.

**Alternatives.** Ramping the first rows in with a time warp: smoother, but a change to the verified motion.

**Revisit if** step 5 shows a tracking error or a force on the bar beyond the reference run's.

Pinned by `test/Program/test_core.py::test_the_fork_paths_follow_the_recorded_rows_and_join_rule`.

## The program check holds the taught moves to the limits and reports the recorded paths

**Decision.** `Program.Check_Trajectory` (F8.5) measures exact distances at every sample where anything may have moved more
than `Clearance.Check_Step` (1 mm) since the last measured one, and at every segment end; it fails a pair within the 3 mm
clearance unless the step allows its group, and an allowed pair deeper than the group's `Allowed_Depth`. It holds every MoveJ
and MoveL to the `Motion` limits, allowing only a MoveJ's rounding to whole 4 ms samples (the legacy rule rounds T: speeds up
to 1 + 0.5 / n, accelerations its square) and 0.1 % for finite differences; the recorded fork paths' peaks are reported, not
limited. MG06 moves, during a MoveJ / MoveL, from its state at the motion's start to the target's `Fixture` along the
motion's time law.

**Why.** Measured on the default program (2026-10-24): MoveJ peaks reach 1.0031 of the acceleration limit through the
rounding alone; the recorded paths reach 2.03x the assumed acceleration (`PATH_FORK_CLOSE`) and 8.44x at `PATH_FORK_OPEN`'s join
(the velocity step) - the verified physics ran them (SPEC §4.6), so they are the reference, not a fault of the build. With
MG06 switched to the settled closed state at the start of the bar-centring MoveL (Part A's rule) the fork stood 5.95 mm in
the bar (F6's C1 effect); the handle really settles as the fork lets go. Holds are skipped by the motion bound: 10 725 of
17 214 samples measured, check 8 s.

**Alternatives.** Every sample measured: the same result, about 1.6x the time. A fixed 1 % speed / acceleration tolerance:
would hide a MoveL 1 % too fast. The recorded paths held to the assumed limits: the verified motion would fail.

**Revisit if** the acceleration limits are measured on the real robot (KNOWN_ISSUES), or the fork paths are replaced.

Pinned by `test/Program/test_core.py::test_the_default_program_passes_its_check`,
`::test_a_movej_over_its_rule_limits_is_reported`, `::test_contacts_are_allowed_only_where_declared_and_as_deep_as_configured`,
`::test_a_colliding_step_is_reported_with_its_pair_and_time`, `::test_the_handover_keeps_the_board_off_the_ledges`,
`::test_a_press_at_the_handover_is_reported`.

## The default program lifts the PCB clear of the crate and returns from MG06 over the family switch

**Decision.** Two changes to the legacy sequence, found by the program check (F8.5): the pick lifts the carried PCB to a new
auxiliary keypoint `P_INPUT_CLEAR` (170 mm above the PCB's place: the board's bottom 25 mm above the crate's 149 mm walls)
instead of `P_INPUT_ABOVE` (120 mm, the board still below the walls); and the return from MG06 is `P_MG06_ABOVE` →
`P_VIA_HOME_MG06` → `P_HOME`. *Corrected 2026-10-24: Part A said the legacy plan returns "above_handle → home directly", and the
owner's answer 10 ("straight home, as in the legacy plan") rested on that. The reference plan's speed minima show the legacy
planner's collision fallback inserted `P_VIA_HOME_MG06` on the return too (41.156 s P_MG06_ABOVE, 42.404 s P_VIA_HOME_MG06,
43.824 s home); the direct MoveJ drives the fork 8.41 mm through the handle bar.*

**Why.** Measured: with the 120 mm lift the carried PCB passes 1.64 mm from the input crate's wall during the MoveJ to
`P_VIA_INPUT_LOADER` (the legacy plan's own motion; the legacy check, based on generated contacts, could not see it); with
`P_INPUT_CLEAR` nothing comes within 10 mm. The keypoint table keeps its 22 rows byte-identical and gains one row (PASS).
Cost: +0.388 s (the handover's vacuum off at 9.912 s instead of 9.524 s); the return over the via +0.6 s.

**Alternatives.** Allowing `pcb_crate` in `S3_To_Loader`: would hide a real near-miss. A MoveL from `P_INPUT_ABOVE` to the via:
a diagonal climb out of the crate, not a straight lift.

**Revisit if** the crate is lower or the PCB smaller, or the owner wants the straight home move with another via.

Pinned by `test/Program/test_core.py::test_the_default_program_passes_its_check`,
`test/Keypoints/test_core.py::test_every_point_passes_the_check`.

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

## The cell program is three files: Constants, Simulation, Program

**Decision.** The program of the cell is split by who uses each value (owner, 2026-10-24, `FEATURES.md` F10):
`Configuration/Constants.yaml` declares every named value the cell's controllers use (tools, points, loader positions,
gripper commands, recorded paths, I/O signals); `Configuration/Program.yaml` is what the controllers run (the robot's
segments `S<n>_<Name>`, the loader's and the tester's jobs `J_<NAME>`), one call per line; `Configuration/Simulation.yaml`
holds what only the simulation needs (the start state, the model site of each tool, the actuator of each output, the
response time of each input, where the jaws really stop, the settle time, the contacts per segment, each point's scene).
`Configuration/Cell.yaml` stays the cell's physical description. `src/Configuration/Program_Parser.py` is the only reader
of the three new files and builds a typed intermediate representation (`Program_Str`) that everything else reads; the
format change kept every number: the trajectory is byte-identical to the F8 build (the golden test below).

**Why.** The program a person transfers to the real cell must not carry simulation data (contacts, scenes, simulated
signal times), and a value used in several lines is declared once under one name. A typed IR checked at load turns every
naming or grammar error into one message with its file and line, before anything is built.

**Alternatives.** The F8 files (`Keypoints.yaml` + `Program_Default.yaml`): keypoints mixed robot data with scene data, and
the program mixed the controllers' commands with simulation settings. One file for everything: the program could not be
read without the simulation's data.

**Revisit if** a second robot program shares the cell, or the real controller's own file format is adopted.

Pinned by `test/Program/test_golden.py` and `test/Configuration/test_program_parser.py::test_the_shipped_files_load`,
`::test_signals_contacts_and_constants_are_complete`.

## Constants carry typed names; Joints are authoritative, Pose is a check value

**Decision.** Every constant is UPPERCASE with its section's prefix (`T_`, `P_`, `L_`, `G_`, `PATH_`, `DO_` / `DI_`), each
with a one-line `Description`. A point's last word is its role (`ABOVE`, `APPROACH`, `PICK`, `PLACE`, `EXIT`, `CLEAR`,
`WAIT`, `VIA`, and the MG06 fork operations; the name table in SPEC §4.8). A point stores `Joints` [deg] - the pose and the
IK family, the only value playback reads - and `Pose` [x, y, z m; R, P, Y deg], the tool frame in the robot base frame as
fixed-axis roll, pitch, yaw (R = Rz(Y)·Ry(P)·Rx(R)), which must equal FK(Joints, Tool) within 1e-5 m and 1e-3°
(`Keypoints.CONST_POSE_POSITION`, `CONST_POSE_ANGLE`), compared as rotations, never as numbers; each angle is written in
(-180, 180] (one spelling per value, owner 2026-10-24: a roll of -180 is written 180; the parser refuses another, and
`resolve_keypoint.py` prints it so - `Kinematics.Degrees_Text`). A loaded name is checked against the kind its argument needs (a
MoveL takes a `P_`, a MoveY an `L_`).

**Why.** A typed name says what it is in every line that uses it, so a wrong kind is a load error, not a run-time surprise.
`Pose` is what a person reads on a teach pendant; keeping it as a checked copy of the joints makes the file readable
without making it a second source of truth. The 1e-5 m / 1e-3° tolerance is the rounding of the printed values (9 decimal
places in metres and degrees), far below the stale tolerance.

**Alternatives.** Pose as the source with IK at load: the IK family and branch become a run-time choice (F6's determinism
rule). Lowercase names (F6, F8): no kind in the name.

**Revisit if** the RPY convention differs from the AUBO controller's (`KNOWN_ISSUES.md`), or points are taught on the real
cell (their joints then come from the controller).

Pinned by `test/Keypoints/test_core.py::test_every_pose_is_fk_of_its_joints`, `::test_a_pose_that_is_not_fk_is_reported`, `::test_a_pose_is_compared_with_fk_as_a_rotation`,
`test/Kinematics/test_core.py::test_rpy_angles_are_normalized_and_compared_as_rotations`,
`test/Program/test_core.py::test_the_shipped_constants_match_the_model`, `::test_the_constants_are_checked_against_the_model`,
`test/Configuration/test_program_parser.py::test_an_invalid_constant_is_named_with_its_file_and_line`.

## A derived point is its Reference moved by an Offset in the Reference's tool frame

**Decision.** A point derived from another carries `Reference: <P_ point>` and `Offset: [dx, dy, dz]` [m], the
displacement in the **Reference point's tool frame** (for `T_SUCTION`, +Z from the flange towards the cups: a negative dz
is above, away from the part). Both fields are required on every point, both `null` on a root point; one set without the
other, an unknown or non-point Reference, and a Reference cycle are load errors naming the file, line and point. The
point's FK must equal the Reference's pose moved by the Offset within `Clearance.Stale_Position` / `Stale_Angle` (0.05 mm,
0.01°); its `Joints` stay authoritative and `Example/Keypoints/resolve_keypoint.py` prints corrected `Joints` and `Pose`
lines. The roots that are placed against the PCB (`P_INPUT_PICK`, `P_LOADER_PLACE`, `P_OUTPUT_PLACE`) take their target
from `Simulation.yaml`'s `PCB_Hold` (a PCB location and an offset), since the PCB's poses are simulation data. The shipped
offsets are within 0.0387 mm of their Reference (measured 2026-10-24).

**Why.** An approach is taught as "15 mm before the part along the tool": the tool frame of the part's point is the frame
a person thinks in, and it keeps the offset valid when the place is re-taught. The names `Reference` / `Offset` (owner,
2026-10-24) replace F6's single mapping of a target, a frame and an offset.

**Alternatives.** The offset in the base frame: wrong as soon as the place is tilted (the loader's plate is). The offset in
the derived point's own frame: circular when the derived point is re-resolved.

**Revisit if** a station user frame is introduced (`KNOWN_ISSUES.md`): an offset may then be given in it.

Pinned by `test/Keypoints/test_core.py::test_the_derived_points_are_not_stale`, `::test_a_stale_point_is_reported_and_resolved`,
`test/Configuration/test_program_parser.py::test_an_invalid_constant_is_named_with_its_file_and_line`,
`::test_an_offset_cycle_is_rejected`.

## Program lines are parsed, never evaluated, by a strict YAML loader

**Decision.** A program line is a call (`Robot.MoveL(P_LOADER_PLACE, v=0.03)`) parsed with Python's `ast` into a name,
positional constants and keyword literals, never evaluated; anything else (an expression, an attribute chain, a call as an
argument) is a grammar error. The YAML is read by one loader (`Configuration.Core.Read_YAML`) that rejects duplicate keys,
treats only `true` / `false` as booleans (`ON`, `OFF`, `Open` stay text) and keeps every string's line for the messages.
The only literal values in the program are speeds, durations, waits and timeouts; every other argument is a declared
constant of the kind the command needs. `a=` and `r=` (acceleration, blend radius) are accepted only as 0 or omitted.

**Why.** A teach-pendant-like line is easy to read and to transfer, and parsing it as code without running it removes any
chance of executing the file. PyYAML's defaults silently take the last of two duplicate keys and read `ON` / `OFF` as
booleans; both would hide an error in a hand-written file.

**Alternatives.** One mapping per command (F8): verbose and far from a pendant program. A regular-expression parser: weaker
on nesting and quoting than `ast`, with no gain.

**Revisit if** blending or accelerations are implemented (F9 or later), or a real controller's syntax is adopted.

Pinned by `test/Configuration/test_program_parser.py::test_a_command_line_is_parsed_never_evaluated`,
`::test_the_yaml_loader_rejects_duplicate_keys_and_keeps_on_off_as_text`,
`::test_an_invalid_program_line_is_named_with_file_segment_line_and_text`, `::test_a_program_using_a_value_v_and_blank_a_r_loads`.

## Loader.Run starts a job and waits; Start and WaitDone are reserved for F9

**Decision.** The robot's program is the master of the cycle. `Loader.Run(J_)` and `Tester.Run(J_)` run a station's job and
wait until it is done (Run = Start + WaitDone), so the default program stays sequential. `Loader.Start` / `Loader.WaitDone`
and `Tester.Start` / `Tester.WaitDone` are reserved names: the parser refuses them with that reason until F9 uses them for
parallel motion under interlocks. The loader's job runs one axis after the other (owner, answer 1).

**Why.** The real cell's loader is a PLC with its own program; jobs keep its steps in one place and the robot's program
says only when it starts and waits. Reserving the names now keeps the format stable for F9.

**Alternatives.** Loader commands inline in the robot's program (F8): the PLC's part could not be read as one unit.

**Revisit if** F9's synchronisation needs more than start and wait (an interlock signal, a timeout).

Pinned by `test/Configuration/test_program_parser.py::test_an_invalid_program_line_is_named_with_file_segment_line_and_text`,
`::test_an_invalid_job_line_is_named`, `::test_the_commands_are_resolved`.

## Simulated I/O and the end settle live in Simulation.yaml

**Decision.** `Robot.SetDO(DO_VACUUM, ON)` drives the output's simulated actuator (`Simulation.yaml Actuators`), and
`Robot.WaitDI(DI_VACUUM_OK, timeout=1.0)` holds for the input's simulated response (`Signal_Response`, 0.3 s: the legacy
planner's hold after vacuum on) and fails the load when the response exceeds the timeout. The F8 program's last step
(a 0.8 s wait for the physics to settle) is `End_Settle` in `Simulation.yaml`, appended after the last segment: the
trajectory lasts 68.852 s as before, the program a controller would run 68.052 s. The signals' `Port` is a placeholder
(`KNOWN_ISSUES.md`).

**Why.** A real program waits for its vacuum sensor, not for a time; the time it takes is a property of the simulated cell.
A settle wait for the physics run means nothing to a controller.

**Alternatives.** A plain `Wait(0.3)` after the vacuum: correct in time, but the real program would not check its vacuum.

**Revisit if** the real cell's I/O map is defined, or step 5's physics measures the vacuum build-up.

Pinned by `test/Program/test_core.py::test_a_waitdi_holds_its_simulated_response_and_the_cycle_ends_with_the_settle`,
`test/Configuration/test_program_parser.py::test_signals_contacts_and_constants_are_complete`.

## Every constant is used; no field without effect

**Decision.** A constant declared in `Constants.yaml` and used by neither `Program.yaml` nor `Simulation.yaml` is a load
error, and so is a job no segment runs. So `T_FORK` is not declared (the fork's MoveLs and paths run on the flange,
`T_FLANGE`), and `Cell.yaml Clearance.Check_Position` was removed: the old `Check` value it bounded is replaced by the
`Pose` check (1e-5 m), and nothing read it any more.

**Why.** project-style-rpx: no configuration field that cannot take effect; an unused constant is either a mistake or dead
data a reader would trust.

**Alternatives.** Declaring `T_FORK` for documentation: a tool no line uses.

**Revisit if** a command runs on the fork's site.

Pinned by `test/Configuration/test_program_parser.py::test_signals_contacts_and_constants_are_complete`.

## The program's model checks live in the Program layer

**Decision.** `Program_Parser` checks everything that needs no model: names, kinds, grammar, references, cycles, the family
switch, the path ends, completeness. The checks against the compiled model - every tool's `Flange_Offset` against its site,
every `Pose` against FK, every derived point against its target, the loader positions and gripper commands within the
model's ranges - are `Program.Validate_Constants`, run first by `Build_Program`, each problem with `Constants.yaml`'s line.

**Why.** The layer order (SPEC §6) keeps `Configuration` free of MuJoCo and of the project's other domains; the model checks
need `Kinematics`.

**Alternatives.** Kinematics in `Configuration`: breaks the layer order and makes every configuration test load MuJoCo.

**Revisit if** the layer order changes.

Pinned by `test/Program/test_core.py::test_the_constants_are_checked_against_the_model`, `::test_the_shipped_constants_match_the_model`.

## The format change was proved against a golden build

**Decision.** Before the format change, the F8 build (`e5ec42c`) was captured in `test/Program/data/` (commit `e77c004`):
the sha256 of every trajectory array's bytes, the cycle time, the events, the segments, each command's target joints as
`float.hex`, the check report and the keypoint table. `test/Program/test_golden.py` rebuilds the program in the new format
and compares: every numeric array byte-identical, the same events and cycle time (68.852 s), the 69 joint targets
bit-identical, the report and the table identical after the old → new name map (`test/Program/data/name_map.json`). The
table's check and stale columns are the exception: they now measure the `Pose` against FK and the `Reference` / `PCB_Hold`
consistency, by design.

**Why.** A format change must not move a number; byte identity is the strongest proof and cheap to keep.

**Alternatives.** Tolerance comparisons: would let a real change through. Comparing only the cycle report: blind to the
trajectory's internals.

**Revisit if** the program's behaviour changes on purpose (F9, a re-taught point): the golden data is then re-captured in its
own commit with the reason.

A change that moves the numbers on purpose re-captures the golden arrays in its own commit, with the samples that changed
listed: F9.6 (MoveJ ends exactly on its stored joints) changed robot_q at 6073 samples by at most 1.55e-15 rad (6.2 ulp),
robot_qd at 2395 by at most 3.0e-13 rad/s (the central differences of those roundings) and pcb_pose at 811 by at most
8.5e-16 m; every other array, the events, the cycle time, the check report and the point table stayed the same.

Pinned by `test/Program/test_golden.py`.

## F9 drops the multi-seed IK of a FREE segment's ends

**Decision.** F9's pipeline does not search several IK solutions for a FREE segment's start and end (owner, 2026-10-24,
answer 5). A FREE segment runs between the ends of LOCKED segments, whose stored joints are fixed; there is nothing for a
seed search to choose.

**Why.** A different IK solution at an end would change the LOCKED segment it joins, which must stay byte-identical.

**Alternatives.** Multi-seed IK with a free end pose: would re-teach process-critical points.

**Revisit if** a FREE segment ever ends at a point no LOCKED segment fixes.

Pinned by nothing yet (F9 is designed, not built).

## The viewer loop holds the lock only while posing; the overlay is set outside it

**Decision.** A passive-viewer playback (`Render.Run_Viewer_Loop`) holds `viewer.lock()` only while it writes the viewer's
data (posing); `set_texts` and `sync` are called outside it. The overlay is set only when its text changes: at once on a
segment change, otherwise at most every 0.25 s (`CONST_OVERLAY_PERIOD`). `sync` runs every frame, paused too; the frame
period is 1/60 s and a slower frame skips samples by the wall clock (`Playback_Cls`); keys are queued by the viewer's
thread and applied at the start of the next frame. `--watchdog S` dumps every thread's traceback and exits 3 when the loop
stops advancing.

**Why.** mujoco 3.16's `Handle.set_texts` (viewer.py line 168) hands the texts to the render thread and waits until it has
taken them; the render loop (`simulate.render_loop()`, viewer.py line 525) needs the simulate mutex to do so. Called inside
`with viewer.lock():` the script thread holds that mutex while it waits: both threads block and the window freezes
(reproduced 2026-10-24 under mjpython: the owner's traceback; outside the lock `set_texts` returns in 0.1 ms). Rate-limiting
keeps the waits few: the text changes every frame (the time).

**Alternatives.** The overlay drawn through `user_scn` labels: more code for the same text. The terminal line only: loses
the overlay the owner asked for (both kept: the terminal prints each segment).

**Revisit if** mujoco's `set_texts` stops waiting for the render thread, or the viewer loop gains other Handle calls (each
checked against the lock rule).

Pinned by `test/Render/test_core.py::test_the_overlay_is_never_set_under_the_viewer_lock`,
`::test_the_playback_follows_the_wall_clock_and_skips_samples_on_slow_frames`, `::test_keys_arriving_mid_frame_apply_at_the_next_frame`,
`::test_the_overlay_updates_at_most_four_times_a_second_and_at_once_on_a_segment_change`,
`::test_a_stalled_loop_is_reported_by_the_watchdog`,
`test/Example/Program/test_show_program.py::test_a_frame_of_the_playback_fits_the_frame_period`.

## The planner and the optimizer are two domains

**Decision.** Joint-space planning is `src/Planner/Core.py` (the spline, TOPP, RRT-Connect, the shortcut, the prune, the
refit; numpy and scipy only, a configuration's validity a function the caller gives); the optimized program is
`src/Optimizer/Core.py` (the FREE transfers on the cell, the program written, its verification). `Program` imports
`Planner` (a MoveSpline is timed at build); `Optimizer` imports `Program` (it builds and checks what it writes).

**Why.** The approved design put both into `Planner`; then `Program` (timing a MoveSpline) and `Planner` (building the program)
would import each other. Split, the layer order stays a tree (SPEC §6) and the planner is tested without the cell.

**Alternatives.** The spline timing copied into `Program`: two TOPPs to keep equal.

**Revisit if** another program form needs the planner outside `Program`.

Pinned by `test/Planner/test_core.py`, `test/Optimizer/test_core.py`.

## TOPP is our own, by reachability with first-order interpolation

**Decision.** The time-optimal timing is `Planner.Topp`: TOPP-RA's reachability (Pham & Pham 2018) on a uniform grid, `s̈`
constant per interval, its acceleration limits met at both ends of each interval (first-order interpolation: `x_{i+1} = x_i
+ 2 Δs u_i` substituted at the end point), every bound on `u` linear in `x`, so the controllable set is an interval found in
closed form from the pairs of bounds; the timing is sampled exactly (`s` quadratic in time per interval), stretched to whole
4 ms samples and further while a sample exceeds a limit between grid points.

**Why.** toppra has no macOS arm64 wheel (FEATURES.md F9). Collocation at the grid points alone left `u` unbounded where a
clamped spline's `q'` vanishes (its ends): the sampled acceleration reached 1.75x the limit between grid points and the
stretch made a 1.80 s transfer 3.11 s (measured 2026-10-24); with first-order interpolation the stretch is 0.1-0.3 %. A first
vertex-enumeration LP was replaced by the closed form: with its open bounds of 1e9 it lost precision and the interval
oscillated. B3: the closed-form trapezoid and triangle are met within 1.9e-7 (relative; 1000 intervals).

**Alternatives.** A jerk-limited timing: not approved (answer 13: bang-bang accelerations, step 5 reports the tracking).
**Revisit if** step 5 shows tracking errors from the acceleration steps, or a real controller's limits differ.

Pinned by `test/Planner/test_core.py::test_topp_matches_the_closed_form_of_a_straight_move`,
`::test_a_timed_spline_keeps_the_limits_at_every_sample`.

## The planner's path keeps a wider margin than its spline

**Decision.** RRT-Connect and the shortcut keep the clearance plus a planner margin (`Optimization.yaml Margins`, the first
2 mm, then 3 and 5 mm when a transfer fails the exact check); the spline through their path is checked against the
clearance plus `Spline.Margin` (1 mm) and refitted where it fails.

**Why.** Planned at the spline's own margin, the polyline grazes the obstacles and the spline through it cuts them: the
carried PCB's transfer to the output crate stayed invalid after 80 refits at 1 mm; with the polyline at 2 mm it passes after
2 refits. 10 mm would make `P_LOADER_ABOVE` itself invalid (measured 2026-10-24). The final word is the exact check at every
sample.

**Alternatives.** One margin for both: the spline needs room the polyline does not leave.

**Revisit if** a transfer fails at every margin.

Pinned by `test/Optimizer/test_core.py::test_the_free_space_keeps_the_clearance_and_the_tilt`,
`::test_the_optimized_program_passes_every_check_and_is_faster`.

## A transfer's place among its parallel jobs comes from the exact robot-loader check

**Decision.** A FREE transfer with loader jobs between its taught segments starts the first job, runs its spline, then
waits for each job and starts the next. When the build's check finds the robot within the clearance of the loader (I8), the
spline moves after the start of the next job if the meeting happens while a later job runs (the robot would stand at its end
point then), otherwise it waits longer before the spline by the length of the meeting window; a meeting while the robot waits
at its start fails, naming the transfer. Shipped: To_MG06 runs its spline during `J_INSERT`; To_Take after the start of
`J_RETURN` - with the spline during `J_FETCH` the robot standing at `P_LOADER_ABOVE` meets the `z_carriage` (-18.91 mm) while the
loader comes back at its travel height (measured 2026-10-24).

**Why.** In the taught program the robot is at home while the loader fetches and returns; `P_LOADER_ABOVE` is in the
loader's travel height path. Waiting longer could not help there (the meeting moved with the wait); a later slot does.

**Alternatives.** Planning the robot against the loader's swept volume (the design's conservative variant): it would forbid
`P_LOADER_ABOVE` as an end point outright. A waiting point between: a second spline and a stop.

**Revisit if** the loader's positions or jobs change (the optimizer finds the slot again).

Pinned by `test/Optimizer/test_core.py::test_the_interlocks_hold`, `test/Example/Program/test_optimize_program.py`.

## The optimized program is generated data, committed and reproducible

**Decision.** `Data/Optimized/Program_Optimized.yaml` is written only by `Example/Program/optimize_program.py` and committed.
It records the sha256 of its inputs (the four configuration files, `Optimization.yaml`, the fork path, the compiled planning
model), the platform and versions, each segment's start time and the TOPP grid. A build refuses it when an input changed or
when it does not reproduce the schedule; the optimizer run again writes the same bytes (B5).

**Why.** The planning is random with a fixed seed: the file keeps the result reviewable and diffable, and the hashes keep it
honest when the taught program or the cell changes.

**Alternatives.** Planning at every build: slow (34 s) and the result unreviewed.

**Revisit if** the optimizer runs on another platform (its output may then differ: re-run and commit with the platform).

Pinned by `test/Example/Program/test_optimize_program.py::test_the_optimizer_reproduces_the_committed_program_byte_for_byte`,
`test/Optimizer/test_core.py::test_the_shipped_optimized_program_is_current_and_matches_its_schedule`,
`test/Example/Program/test_build_program.py::test_the_optimized_program_is_built_and_a_stale_or_edited_one_refused`.

## Run is Start plus WaitDone; a robot Wait holds only the robot

**Decision.** The builder runs a started loader job one sample per robot sample (a motion, a `Wait`, a `WaitDI` of the robot
all advance it); `Loader.WaitDone` holds the robot until the job's last sample. `Loader.Run` keeps its expanded form and gives
the same samples as Start + WaitDone. In the taught program no job runs during a robot line, so its trajectory is unchanged
(golden).

**Why.** A real master program runs this way: the PLC's job goes on while the robot works.

**Alternatives.** A separate loader timeline merged afterwards: the PCB rules (clamp at MG06, release) would need the merge to
know both lanes' states.

**Revisit if** the tester's job runs in parallel too (Tester.Start is reserved).

Pinned by `test/Program/test_core.py::test_a_run_is_a_start_and_a_waitdone`, `test/Optimizer/test_core.py::test_the_loader_moves_while_the_robot_moves`.

## The optimized program's longer PCB path is accepted

**Decision.** The optimized program carries the PCB farther than the taught one: 3.010 m against 2.712 m (+11 %) with the first
30 deg tilt limit, 2.856 m (+5 %) with 20 deg (measured 2026-10-24), while the robot's joint travel falls 18 % and the TCP's
path 13-15 %: accepted (owner, 2026-10-24).

**Why.** The optimizer's objective is time within the limits and the clearance; the FREE transfers that carry the PCB (To_Loader,
To_Output) are shorter in time (1.148 s against 1.836 s, 1.768 s against 2.556 s), their clearances pass, and the tilt stays
within its limit.

**Alternatives.** A path-length term in the planner: no gain in time, more tuning.

**Revisit if** step 5 shows PCB slip in a carrying transfer.

Pinned by `test/Example/Program/test_compare_programs.py`.

## A robot stop is a tenth of a second at rest

**Decision.** The comparison counts a robot stop when every joint speed (its change per sample over 4 ms) stays below 1e-3 rad/s
for at least 0.1 s after a motion (`Optimizer.CONST_STOP_SPEED`, `CONST_STOP_TIME`), and reports the time at rest in them.
Legacy, taught and optimized all stop 11 times; at rest 18.600 / 21.176 / 16.716 s (measured 2026-10-24).

**Why.** The first count (a sample moving followed by one at rest) counted the legacy plan's single still rows: 135 stops. With
a duration, the count is the process's holds - the vacuum at the pick, the handover's clamp, the fork paths' recorded rests,
the test, the take, the place - and the waits for the loader; the optimization removes motion and moves the waits, so the count
stays and the time at rest falls.

**Alternatives.** A speed threshold alone: still counts the transient zero speed of a min-jerk move's ends.

**Revisit if** the comparison should tell the process holds from the waits.

Pinned by `test/Optimizer/test_core.py::test_a_stop_is_a_tenth_of_a_second_at_rest_after_a_motion`,
`test/Example/Program/test_compare_programs.py`.

## The optimized take waits at P_LOADER_WAIT; the loader is the critical path

**Decision.** In the optimized program the robot leaves `P_MG06_FORK_START` at once after Open_MG06 (with `J_FETCH`), waits at
`P_LOADER_WAIT` (25 mm above `P_LOADER_ABOVE` along the plate normal, `Optimization.yaml Waiting_Points`) and moves on to
`P_LOADER_ABOVE` at the earliest moment the exact robot-loader check allows (1.660 s after `J_RETURN` starts). Interlock I7
stays: the take starts when `J_RETURN` has ended.

**Why.** Owner's observation (2026-10-24): the robot waited 4.05 s above MG06. The analysis: the loader is the critical path -
`J_FETCH` 37.036-41.080 s, `J_RETURN` 41.080-44.680 s (at rest at `L_HOME` from 44.480 s, then its Wait(0.2)) - and the robot was
already at `P_LOADER_ABOVE` at 43.364 s, so leaving earlier gains no time: the cycle stayed 57.132 s (57.108 s since the 20 deg
tilt limit). It removes the robot's wait
above the tester (which a real cell's safety zones would have had to allow). `P_LOADER_ABOVE` itself cannot be the waiting
point: the `z_carriage` at its travel height meets the robot there (-18.91 mm); 23 mm above keeps 3.26 mm, 25 mm 4.92 mm (3 mm
plus the planner's first margin), 20 mm 0.85 mm (measured over every sample of `J_FETCH` and `J_RETURN`). I7' (the take on the
loader at rest at home, during its trailing Wait(0.2)) would gain at most 0.2 s and needs a state-based "loader home" input and
a WaitDone inside the LOCKED take: not done. Other candidates (measured): the wait before Close_MG06 (the robot at
`P_MG06_FORK_START` 12.828-17.924 s while `J_PARK` runs) - loader-limited by I3, gain 0; starting `J_INSERT` during the
handover's last MoveL (9.224-10.720 s) - at most 1.5 s, but it splits a LOCKED segment and relaxes I2: not done without a
decision. The closest robot-loader distance is now 4.28 mm at 42.796 s (the last leg behind the passing `z_carriage`).

**Alternatives.** One spline straight to `P_LOADER_ABOVE` timed after the loader passes (the previous program): the same time,
the wait above MG06.

**Revisit if** the loader's jobs get faster (the robot may become the critical path), or I7' / I2' are wanted.

Pinned by `test/Optimizer/test_core.py::test_the_robot_leaves_mg06_at_once_and_waits_at_p_loader_wait`, `::test_the_interlocks_hold`,
`test/Configuration/test_program_parser.py::test_an_optimizer_point_is_used_only_by_the_optimized_program`.

## Step 5 plays a trajectory through the cycle run's loop (design, F11)

**Decision (proposed, awaiting approval).** The physics playback of a program reuses `Simulation.Run_Cycle`'s loop unchanged
(servo set-points with velocity feed-forward, the filtered loader drives, vacuum adhesion, the instability guard, every
measurement), fed through a drive object: `Legacy_Drive_Cls` reproduces today's run from `cell_plan.npz` (its report
byte-identical, the old baseline), `Trajectory_Drive_Cls` reads a trajectory file (the taught or the optimized program's).
What the legacy run took from its phase labels comes from the program instead: the fork's allowed contact from the steps'
`Allow_Contact`, the "placed" and "closed" marks from `J_PARK` and the `Test` step.

**Why.** One loop keeps one physics: the programs are measured exactly as the reference plan was, and the legacy report proves
the refactor.

**Alternatives.** A second physics loop for trajectories: two loops to keep equal. Converting a trajectory into the legacy plan
format: loses the analytic speeds and the steps.

**Revisit if** the owner's answers to F11's questions change the marks or the fork rule.

Pinned by nothing yet (designed, not built).

## The carried PCB tilts at most 20 deg

**Decision.** `Optimization.yaml Carry_Max_Tilt` is 20 deg (owner, 2026-10-24), not the first, assumed 30 deg.

**Why.** It costs no time: with 20 deg the optimizer plans To_Output at 17.73 deg in 1.768 s (30 deg: 28.98 deg in 1.792 s), the
cycle 57.108 s (57.132 s) - the tighter limit steered RRT-Connect to a slightly shorter path; a flatter board is less likely to
slip on the cups (KNOWN_ISSUES.md).

**Alternatives.** 30 deg: the same time, a board tilted almost 29 deg while it accelerates.

**Revisit if** F11's physics run shows slip in a carrying transfer (or none, which would allow more), or a real-cell value
exists.

Pinned by `test/Optimizer/test_core.py::test_the_carried_pcb_keeps_its_tilt_limit`.
