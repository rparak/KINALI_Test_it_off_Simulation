# Design decisions

Deliberate choices whose reason is not visible in the code. Each one says what would reopen it.

## The project follows project-style-rpx and the UR_PathGen_SF_1 layout, not the brief's package layout

**Decision.** The code, tests, scripts, installer and records follow the personal skills
(project-style-rpx first) with github.com/rparak/UR_PathGen_SF_1 as the structural template:
`src/<Domain>/Core.py` without `__init__.py`, `Example/` scripts, `test/` mirroring `src/`,
`install.sh` + `requirements.txt`, `.claude/project-docs/`. This replaces the layout of the brief §5.1
(package `kinali_sim`, `pyproject.toml`, console script `kinali-sim`, `tests/`, `config/`) wherever they
conflict (owner, 2026-10-24). Where the skills and the template disagree, the skills win and the
difference is recorded (the style project reference > Departures).

**Why.** One convention across the owner's projects; the skills carry the checker, the test discipline
and the records that keep a verified simulation verified.

**Alternatives.** The brief's package layout: rejected by the owner for consistency. Copying the template's code: rejected - only its structure and
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

Pinned by `test/Keypoints/test_core.py::test_every_pose_is_fk_of_its_joints` (the joints' unit and order, the TCP),
`test/Trajectory/test_core.py` (the trajectory's metadata, its units included, kept through the file) and the README alert;
the joint order and the TCP the metadata records are `Program.Core.CONST_META`.

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
- where: the README header;
- how: black SVG on light grounds, white SVG on dark grounds (GitHub dark mode),
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

*Corrected 2026-10-24: the Test-it-off logo is not used (owner).* *Corrected 2026-10-25 (owner): no PyQt GUI, so the
application header is gone; jic-sf-ui-style remains the source of the JIC logo and its rules.*

**Why.** One monochrome family reads on both grounds and beside each other; the owner's decision.

**Alternatives.** The JIC purple original as jic-sf-ui-style prescribes ("the purple version is for white
grounds"): overruled by the owner until the official monochrome file arrives.

**Revisit if** JIC Smart Factory supplies the official monochrome vector, KINALI supplies one, or the brand
rules change.

Pinned by `test/Example/Media/test_make_logo_variants.py` (originals' hashes, one colour, geometry kept,
byte-identical regeneration).

## The project is named "Test-it-off Simulation"

**Decision.** The project name is **Test-it-off Simulation**; the full title
**Test-it-off Simulation – Automated PCBA Testing Cell in MuJoCo**. It is used in the README title, SPEC's title and §1 and
`CLAUDE.md`. The official spelling of the product is **Test-it-off** (with
hyphens) everywhere except verbatim quotes of external sources and the URL testitoff.cz. The repository
name `KINALI_Test_it_off_Simulation` stays unchanged (owner, 2026-10-24).

**Why.** One name for the project in every place a reader meets it; KINALI's own spelling of the product.

**Alternatives.** The repository name as the project name: rejected, it is a technical identifier.

**Revisit if** KINALI changes the product's name or spelling.

Pinned by: none (text); checked by `git grep` for the old spellings when docs change.

## No PyQt GUI (decided, owner 2026-10-25)

**Decision.** The project has no desktop GUI application: no PyQt5 or PyQt6 window and no Phase 2; the project is one phase.
Its interface is the scripts of `Example/`, the MuJoCo viewer (`show_program.py`, `show_physics.py`, the keypoint viewers)
and the offscreen renders. *Corrected 2026-10-25 (owner): this entry was "Phase 2 GUI toolkit: PyQt5 or PyQt6 - open".*

**Why.** The owner's decision: the results are read in the viewer, the reports and the README.

**Alternatives.** PyQt5 with jic-sf-ui-style's ready package (the brand at no cost; Qt 5.15 is the end of Qt 5); PyQt6 as
the brief §8 proposed (the brand package ported). Both dropped.

**Revisit if** the owner asks for a GUI application; the toolkit choice is then reopened with these two options.

Pinned by: `requirements.txt` has no Qt package; `src/Render/Core.py` imports the viewer only inside the functions that
open it (the headless scripts and the tests never open a window).

## The keypoint check measures exact distances, not generated contacts

**Decision.** The keypoint clearance (and, from F8, the program's segment check) is measured with `mj_geomDistance`
for every collidable pair within the clearance (`Collision.Close_Pairs`), not read from the contacts MuJoCo generates in
a model with an enlarged geom margin (the legacy planner's planning model). `Close_Pairs` filters the pairs with a
vectorised bounding-sphere broad phase (numpy, a 1e-9 m slack so it keeps every pair the per-pair test kept), then
measures each candidate exactly: the same pairs and distances as the per-pair loop, 38 times faster (owner's B2).

**Why.** Generated contacts miss near pairs depending on unrelated geometry: the carried PCB 1.02 mm above the crate
floor at `P_OUTPUT_PLACE` produces no contact in the cell with the 6 mm margin, and 16 contacts once one more geom - even a
box 5 m away - is added to the workstation body (measured 2026-10-24): the workstation body's mid-phase decides, so
contact generation is not a reliable distance measure. *Corrected 2026-10-24: first blamed on MuJoCo's box-box collider.*
Exact distances also found the cup block 2.1 mm above a held PCB (now in `cups_pcb`) and the robot's mount (excluded by a
code constant). The broad phase measured on the legacy plan (2026-10-24, Apple M2 Pro): identical pair lists, 15.98 -> 0.42
ms per sample at 3 mm (37.9x).

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

**Why.** The robot and the carried PCB can reach the table; nothing modelled it. The workstation's collidable geoms are the
pedestal and its plate, a frame post (`L_900`), the top beam (`L_340`), the camera holder, the cabinet and a box; nothing
else above the table is within the robot's reach without already being collidable (corner brackets up to z 0.164 m at
x ≈ −0.2, y ≈ −0.8 m, 0.78 m from the base; measured 2026-10-24). Measured: the robot stays at least 8.9 mm
from the boxes over the legacy reference plan; the keypoint table is byte-identical with them; the PCB on a crate floor is
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
leave / return keypoints are settled `Closed` (owner, 2026-10-24).

**Why.** The fork keypoints touch the handle bar, and the contact depends on the handle angle: at
`P_MG06_CLOSE_END` the bar is 0.33 mm into the fork in the recorded state and 5.95 mm in the settled closed
state (measured 2026-10-24). Posing or checking in the wrong state reports a collision that is not there, or
hides one.

**Alternatives.** One MG06 state per program step: rejected, the state changes along the fork path.

**Revisit if** the fork path is re-recorded.

Pinned by `test/Keypoints/test_core.py::test_the_fork_points_in_their_recorded_state_touch_the_bar_at_most_half_a_millimetre`,
`::test_the_close_end_in_the_wrong_mg06_state_is_reported`.

## The contact groups are only those a keypoint or a program step observes

**Decision.** The check knows four contact groups, `cups_pcb`, `handle_bar`, `pcb_crate` and `pcb_jaws` (the PCB on the
loader's jaws), each with its deepest allowed overlap in `Configuration/Cell.yaml` `Clearance.Allowed_Depth` (1.6, 0.6,
0.5, 0.5 mm); a deeper overlap fails even where the group is allowed. `handle_bar` is 0.6 mm (owner, 2026-10-24): the
recorded `PATH_FORK_CLOSE` path overlaps the bar by 0.539 mm at one row - the physics-verified path - measured plus a margin.
A point declares its groups in `Simulation.yaml Points.<P>.Contacts`, a program segment in `Simulation.yaml Allow_Contact`
(owner, 2026-10-24); every declared group must be observed. `pcb_jaws` is observed by the handover and the take, where the
carried PCB is between the closing jaws. *Corrected 2026-10-25: a point's groups are its `Contacts`, not `Allow_Contact`.*

**Why.** A group nothing declares is a field that cannot take effect; allowing only what is present keeps an unexpected
contact visible.

**Alternatives.** A fixed list of every possible group: groups nothing declares would be fields without effect.

**Revisit if** a group is no longer observed by any keypoint or step.

Pinned by `test/Configuration/test_program_parser.py::test_an_invalid_constant_is_named_with_its_file_and_line`,
`test/Configuration/test_configuration_files.py::test_the_shipped_cell_configuration_loads_in_si_units`, `test/Keypoints/test_core.py::test_every_declared_contact_is_really_present`,
`::test_the_contact_groups_of_a_pair`.

## MuJoCo is pinned to 3.16.0

**Decision.** `mujoco==3.16.0` in `requirements.txt`; the installer checks the version.

**Why.** The physics was tuned and verified on 3.16 (SPEC §4.7): in 3.16 `gap` is not applied to
`includemargin`, which is why the PCB and the cups carry no `margin`/`gap`; another version may treat
contacts differently. A pinned version also keeps the baselines deterministic: on one machine and one version both
programs' physics reports are reproduced byte for byte (measured 2026-10-25: the 20 baseline runs, sequential and parallel,
identical).

**Alternatives.** An unpinned or `>=` dependency: rejected, a silent MuJoCo update could change every
contact number and the baseline would no longer be comparable.

**Revisit if** a MuJoCo release is needed for a feature; the upgrade is its own change with a new
baseline, the fingerprint re-measured and the margin behaviour re-checked.

Pinned by `test/Scene/test_core.py` (fingerprint), `test/Simulation/test_baselines.py` (slow regression) and `test/Install/test_install.py` (the version check).

## Python 3.11, pinned requirements, installed by install.sh

**Decision.** The interpreter is Python 3.11 (`/opt/homebrew/bin/python3.11` on macOS, `python3.11` on
Linux); every direct dependency is pinned `==` in `requirements.txt` with the owner's pin list; the
environment is created by `install.sh` into `.venv`. `pyproject.toml` is removed (owner, 2026-10-24).

**Why.** The same versions as the owner's other MuJoCo project, so both share one verified toolchain;
project-style-rpx prescribes the installer and pinned direct dependencies. The interpreter is named
explicitly because a bare `python3` on this machine is miniforge's.

**Alternatives.** The Python 3.14.7 / numpy 2.5.4 environment the first baseline was measured in: rejected by the owner
(the legacy report was byte-identical in 3.11, 2026-10-24); the 3.11 environment is the baseline environment (SPEC §5).
`pyproject.toml` with an editable install: rejected, the skills use no package and no `__init__.py`.

**Revisit if** a pinned package has no wheel for a target, or the owner moves both projects to a newer
Python.

Pinned by `test/Install/test_install.py` (16 tests, sourced with stubs).

## The media are encoded with Pillow and imageio-ffmpeg's pinned ffmpeg

**Decision.** `pillow==13.0.0` (already installed with matplotlib, now a direct dependency) writes the GIF previews, the
step labels and the contact sheet; `imageio-ffmpeg==0.6.0` provides a pinned ffmpeg 7.1 binary that `src/Report/Media.py`
runs as a separate process for the H.264 videos (owner, 2026-10-25). Both are tool dependencies of the media scripts only:
no simulation, check or physics result depends on them.

**Licences** (checked 2026-10-25 on the installed wheels): Pillow is MIT-CMU (HPND); imageio-ffmpeg is BSD-2-Clause; its
bundled binary `imageio_ffmpeg/binaries/ffmpeg-macos-aarch64-v7.1` reports `--enable-gpl` (with libx264 and libx265), so the
binary is GPL-2.0-or-later. The project runs it as a program (raw frames piped in, an MP4 out); it neither links nor
redistributes it (pip installs it from PyPI into `.venv`), so the project's MIT licence is unaffected, and the videos it
writes are the project's output, not covered by the encoder's licence. The README's third-party notes say this when the
README is written (step 7).

**Reproducibility** (measured 2026-10-26: `make_media.py` re-run in a fresh clone against the 2026-10-25 render, Apple M2 Pro). Every
file in `Media/` came out byte-identical (the GIFs: one palette from a fixed strip, no dithering, Pillow's single-threaded quantizer;
the PNGs), except the charts' SVGs, which differ only in matplotlib's random clip and marker ids. Of the 182 files in
`Data/Generated/Reports/Comparison/`, 175 were byte-identical. Three close-view PNGs at the pick moment differed by 1 level on
10-20 of 3.7 million pixels, and the three videos that contain that moment (`Optimized_Mosaic_1440p.mp4`, the Optimized handover and
fork-close clips) differed in exactly one group of pictures (250 frames, libx264's keyframe interval): libx264 itself is
deterministic (the same frames encoded twice give the same bytes), but a 1-level input difference changes its rate-control
decisions until the next keyframe. The cause of the 1-level render difference is not established (the physics reports are
identical). Tolerance for a regenerated video: the same frame count, duration and resolution, and every frame at least 40 dB PSNR
against the previous one (measured: at least 45.9 dB, mean difference at most 0.42 of 255 levels).

**Alternatives.** A system ffmpeg (`brew install ffmpeg`): not installed by `install.sh`, so not reproducible. Pillow only (GIFs,
no MP4): rejected by the owner, the full videos are wanted in high resolution. PyAV or OpenCV's writer: larger, and their
wheels bundle the same GPL/LGPL codecs.

**Revisit if** the project starts to redistribute the binary (a bundled installer, a container image): then the GPL's source
offer applies, or an LGPL build without libx264 is used.

Pinned by `test/Report/test_media.py` (an MP4 and a GIF written and read back).

## The comparison is generated, measured from the runs, every number with its source

**Decision.** `Example/Report/compare_report.py` writes `Data/Generated/Reports/Comparison/Comparison.md` whole (git-ignored), its prose
fixed and every number computed and tagged with its source ([K] the trajectory, [P] the physics run, [C] the configuration,
[M] the model); `src/Report/` holds the measurements (owner, 2026-10-25: a new `Report` domain at the top of the layer order).
The physics side is read from the run by stepping `Physics_Run_Cls` as `Run_Physics` does and reading its data every 4 ms
(`Record_Physics`); `src/Simulation` is not changed.

**Why.** A report edited by hand would drift from the code that measures; a number without its source cannot be checked. A
recorder inside the physics run would change the verified run's code; reading its data between steps cannot change it, and the
run's report proves it (byte-identical to `run_program.py`'s for both programs, 2026-10-25, tested).

**Choices inside.** Exact distances every 20 ms, refined to 4 ms around each pair's minimum, cut at 100 mm (a third of the
distance calls of a 200 mm cut: 71 s for the whole report on the Apple M2 Pro, five processes); zone occupancy from the
collision geoms' points (a mesh's vertices), not their bounding boxes, which make a robot link occupy a zone it only passes
near; the shared zones are the volume the loader's jaws (and the PCB in them) sweep in all its jobs, split by the loader's Y
into the jaw zone (the handover and take position) and the MG06 corridor (travel height to the holder) (owner, 2026-10-25, F16;
replacing F14's box of MG06 over its states, which reached into the loader's home and held the robot most of the cycle, and then
the volume of the whole loader in J_INSERT and J_FETCH, whose part at MG06 held the robot at the handover: the Z carriage reaches
z -0.49..0.25 m, so at the MG06 Y it covers the space above the jaws' home). The carriages against the robot are the gating
rule's and I8's. The robot and the loader are in the corridor together in neither program; in the jaw zone only at the
handover and the take, by design (2026-10-25). The jaws' box at home is kept as the physics cross-check. The poses are one per
5 mm cell of (Y, Z, jaw), so a pose between kept ones is at most 2.5 mm from one along each axis.

**Alternatives.** A hand-written report around generated tables: rejected (drift). A per-step recorder hook in `src/Simulation`:
rejected (it changes the verified run's code for an analysis).

**Revisit if** the report should be committed (then its numbers would need a regression pin) or the run's code changes (the
recorder relies on `Physics_Run_Cls.Step`, `Done`, `Data`, `Result`).

Pinned by `test/Report/test_core.py` (`test_the_recording_does_not_change_the_run`, slow, and the measurements) and
`test/Example/Report/test_compare_report.py` (slow).

## The figure style: Computer Modern, two themes, Okabe-Ito, one colour per program

**Decision.** The comparison's figures follow `src/Report/Style.py` (owner, 2026-10-25): LaTeX-like typesetting from
matplotlib's own Computer Modern (mathtext `cm`, serif `cmr10`, no TeX install, no vendored fonts), a dark (#0D1117) and a light
(white) theme from one definition, the Okabe-Ito colourblind-safe palette for the moving objects, the programs always in the
same two colours - Default: Okabe-Ito sky blue on dark (8.2:1), blue on light (5.2:1); Optimized: the JIC purple tinted 40 % on
dark (5.09:1; the purple itself is 2.07:1 there, below the owner's 4.5:1) and the purple on light (9.15:1) - panel letters, a
300 dpi PNG and an SVG (text as paths) per figure and theme. The programs are "Default program" and "Optimized program" on every
figure, label and heading (owner's addendum). A label's ink on a bar is black or white by the larger contrast.

**Why.** Scientific-paper figures for data analysis, readable on screen and in print, the same colours meaning the same program
everywhere. cmr10 has no bold face and no underscore glyph: panel letters use mathtext bold, names with underscores mathtext
roman (`Style.Label`).

**Alternatives.** The jic-sf-ui-style chart rules: QPainter only, no categorical palette (owner: not ported). A TeX
installation (`text.usetex`): an external dependency for the same look. The dataviz palette: kept only for `compare_programs.py`.

**Revisit if** a figure needs more than eight categories, or the README's background changes.

Pinned by `test/Report/test_style.py` (the contrasts, the label, the files) and `test/Example/Report/test_compare_report.py`.

## The media use named free-camera views on a render-only copy; the GIFs stay under 10 MB

**Decision.** Every medium of `Example/Report/make_media.py` is rendered from a named free-camera view of
`Report.Media.CONST_VIEWS` (lookat, distance, azimuth, elevation, field of view; owner, 2026-10-25): Overview_Right (lookat
(0.33, -0.08, 0.06) m, 3.15 m, azimuth 127 deg, elevation -23 deg, 45 deg; matched to the owner's viewer screenshot),
Overview_Left (its mirror), Top, Input_Crate, Output_Crate, Loader_Handover, MG06_Insert, MG06_Fork. No camera is added to the
model (the fingerprint unchanged); the field of view is set on a render-only copy (`Render_Model`). The views live in
`Media.py`, not a `Report/Const.py`: `Media.py` is their only `src/` reader (the module-layout rule). The videos: H.264, CRF 18,
preset slow, 60 fps, real time, the program, the step, the clock and the line running on a label; the previews: 960 px,
4 x speed, 15 fps on average (GIF frame times are 10 ms steps: 70, 70, 60 ms), one palette from all frames without dithering,
kept under 10 MB by fewer colours, then every other frame; the README's PNGs at most 1.5 MB by a smaller width.

**Why.** Named views make the programs, the points and the stations comparable from the same place; a close view per
station shows what the overview hides (the jaws at the handover, the fork in MG06). One palette avoids GIF flicker.

**Alternatives.** One fixed camera for everything (F14's first version: MG06's fork and the jaws too small to read). Cameras in
the MJCF (they change the model and its fingerprint). The model's `cam_tool` / `cam_pcb` (they follow a body: no common view).
A per-frame palette (flicker).

**Revisit if** a part of the cell must be seen that no view shows.

Pinned by `test/Report/test_media.py` and `test/Example/Report/test_make_media.py` (slow).

## The renders tone down the aluminium; the model keeps its look

**Decision.** `Report.Media.Render_Model` renders a copy of the model whose `look_alu` geoms have their rgb scaled by 0.75 and
the material's specular 0.30 and shininess 0.50 (`CONST_TONE`; the geoms' own rgba is what MuJoCo draws, so the material's
rgba alone has no effect). Only the media use it; `models/`, the viewers and the fingerprint are unchanged (owner, 2026-10-25:
optional, render only). Measured at the home pose, 1280 x 720, pixels with every channel at 250 or more: Input_Crate 3.37 % before, 0.01 % after;
Overview_Right 0.18 %, 0.00 %. The slats under the spot stay the brightest surface in the close views (below saturation).

**Why.** The table's slats burned out to white under the key spot in the overview and drew the eye from the robot.

**Alternatives.** Changing the material in `models/` (changes the model and every viewer; the verified look); a darker
exposure for the whole frame (darkens the robot and the PCB too).

**Revisit if** the studio lights change (F7).

Pinned by `test/Report/test_media.py` (`test_the_tone_down_scales_the_aluminium_geoms_only`).

## The collision structure is drawn over the visual cell in image space

**Decision.** The collision images render the check model (the cell with the planning-only table boxes) twice from the same
view: the visual cell, and the collision geoms only (their groups), each with its depth and geom per pixel. A collision
geom's pixel is blended in its category's Okabe-Ito colour (alpha 0.38) over the visual cell dimmed to 0.88 where it lies on
or before the visual surface (within 12 mm) and outlined where the geom changes (`Collision_Overlay`); a legend names the
categories and their counts. A collision-only image (visuals hidden, opaque colours) is written beside them. All colours are
set on a render-only copy.

**Why.** Blending in image space has no z-fighting, which drawing translucent collision geoms over coincident visual geoms
in one pass has; the depth test keeps hidden geoms out. 12 mm: the offscreen depth on macOS is coarse (no
ARB_clip_control) and the table boxes lie within millimetres of the slats.

**Alternatives.** One pass with translucent collision geoms (z-fighting on every coincident face); collision only (no
context for where a geom is).

**Revisit if** the offscreen depth becomes exact (then the tolerance can shrink to a millimetre).

Pinned by `test/Report/test_media.py` (`test_the_collision_overlay_shows_only_visible_coloured_geoms`).

## The MG06 lock has 1.5 mm radial play

**Decision.** The lock's hook and pin in `models/scene.xml` have 1.5 mm radial play instead of the 0.5 mm of
`mg06_loader_mujoco` (the `GAP` of the MG06 build there).

**Why.** The robot pushes the lid down harder than the free-guided fork of `mg06_loader_mujoco`: with 0.5 mm
the lid sank 0.6 mm into the base, the hook hit the pin already at 27° and the handle never reached 29°.
With 1.5 mm the handle closes to 28.6° (the physics baselines) and the lid rises 0.5° in the play.

**Alternatives.** A softer lid contact: rejected, it would change the verified contact parameters.

**Revisit if** the real lock is measured.

Pinned by the fingerprint (`test/Scene/data/baseline_model_fingerprint.json`) and the physics baselines (MG06's closed
angles +-0.1 deg, SPEC §10.2).

## Logs go to logger/, generated artefacts to Data/Generated/<Kind>/

**Decision.** One place per kind of file a run produces: logs (`logger/<script>.log`, MuJoCo's
`logger/MUJOCO_LOG.TXT`) in `logger/`; generated artefacts in `Data/Generated/<Kind>/` - the reports (a program's
check and physics report, the optimizer's report, the comparison) in `Reports/`, images (`pcb_green.png`, `logo_check.png`,
the comparison's figure) in `Images/`, the programs' trajectories in `Trajectories/`. Both folders are git-ignored and created by `install.sh` and by the scripts; a script's
`--out DIR` moves its file. `models/cell_scene.xml` stays in `models/` (the viewer command loads it); no output folder at the root (owner,
2026-10-24).

**Why.** The root holds only declared entries (style skill); `Data/Generated/` is the template's place for
generated output beside the inputs; logs are not results and keep their own folder. One folder per kind
keeps a report, an image and a trajectory from overwriting or hiding each other.

**Alternatives.** Reports and images in `logger/`: rejected, they are results a reader opens, not run logs.
Keeping an output folder at the root: rejected by the owner.

The programs' trajectories are never committed: they are regenerated deterministically from the committed program files
(`build_program.py`; owner, 2026-10-25).

**Revisit if** a generated file becomes a delivered input (the optimized program is committed in `Data/Optimized/`, its
own decision below).

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

## The offscreen buffer is 2560 x 1440

**Decision.** `Scene.Core._Apply_Look` sets the compiled cell's `visual/global` `offwidth` / `offheight` to 2560 x 1440
(`CONST_LOOK_OFFSCREEN`; `models/scene.xml` keeps its 1280 x 960, the scene file is never edited for the look), so the
comparison's videos render at 1920 x 1080 and 2560 x 1440 (owner, 2026-10-25).

**Why.** MuJoCo refuses an offscreen frame larger than the model's buffer. The buffer is visual only. Proved 2026-10-25: the
fingerprint has its one known difference (`pcb_vis` rgba; it records no visual field), the kinematic golden 0 differences,
both physics baselines 0 failures and 0 report lines changed. The optimized program records the planning model's sha256 of
its binary form (`mj_saveModel`, which includes the visual fields), so `Data/Optimized/Program_Optimized.yaml` was made
again by `optimize_program.py` in the same commit: its one changed line is that hash, every program line byte-identical.

**Alternatives.** The buffer raised on the compiled `MjModel` in the media code only: no stale record, but the model the
viewer and the images use would differ from the one the media use. Rendering tiles and stitching them: needless.

**Revisit if** a larger frame is needed (the GPU's maximum renderbuffer size bounds it), or the optimized program's hash
should ignore visual fields.

Pinned by `test/Scene/test_core.py::test_the_offscreen_buffer_holds_a_1440p_frame`, `test/Render/test_core.py` (a frame beyond the
buffer refused) and the fingerprint test.

## The studio look: fog on, a 1 m grid, a statistic from the cell's bounding box

**Decision.** The scene has the studio look (SPEC §4.10 Look), applied in code (`Scene.Core._Apply_Look`,
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

## Two programs: the default one is taught, the optimized one is planned

**Decision.** Two versions of the same process (owner, 2026-10-24). The **default program**
(`Configuration/Program.yaml`, `FEATURES.md` F8, F10) is written as a human operator would teach it: named
sequential steps, every motion ending at rest at its point, no blending, the robot and the loader never moving
at the same time, approach and retreat points as offsets from keypoints, vias where a person would put them. For
it the brief's rule stands: no planning algorithm, collision freedom is checked, not searched. The **optimized
program** (F9) keeps every process-critical segment of the default program unchanged (LOCKED) and replaces only
the free-space transfers between them (FREE) by planned motion - a joint-space search with a fixed seed, shortcut,
smoothing, time-optimal timing within the robot's limits - and lets the robot and the loader move in parallel
under interlocks. Both versions pass the same program check (SPEC §7.9) and the same physics acceptance (SPEC §10.1).
The brief's "no planning algorithm" (SPEC §2 item 4) is thereby narrowed to the default program; SPEC §2
and §3 say so.

**Why.** The default program is the readable, teachable reference a person can check line by line and transfer
to a teach pendant; the optimized program shows how much of the cycle time is free-space motion and waiting, on
the same process and the same checks, so the two can be compared step by step.

**Alternatives.** One program only, taught (the brief): no measure of the achievable cycle time. One program
only, optimized: nothing a person can read and check against the process; a planner bug would change the
process-critical moves. Optimizing everything (contact moves included): rejected, the contact moves are the
verified physics (SPEC §4.7) and stay as taught.

**Revisit if** the optimized program cannot pass the physics baselines, or the owner decides one version is enough.

Pinned by `test/Program/test_check.py::test_the_default_program_passes_its_check`,
`test/Optimizer/test_core.py::test_the_optimized_program_passes_every_check_and_is_faster` and
`test/Simulation/test_baselines.py`.

## A program's robot points are stored joints; lines and paths end exactly on them

**Decision.** Every robot point of a program is a keypoint's stored `Joints` (F6's determinism rule). A MoveJ interpolates
joints between stored joints and its last sample is the target's stored joints exactly (`q0 + (q1 - q0) x 1.0` would differ by
a rounding, and a LOCKED part after a MoveJ would no longer be byte-identical); a MoveL aims at the pose of its target's stored joints and a `Path` at its recorded rows, with
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
at the cup face. *Corrected 2026-10-24: the first design froze the PCB's pose relative to the TCP at vacuum on; the PCB then
reached `P_OUTPUT_PLACE` 2.5 mm above the crate floor instead of 1 mm.*

**Why.** Each cup sits on a sprung bellows (SPEC §4.10): under vacuum the board is pulled to the cup face as the bellows
extend. The keypoints assume the nominal relation (`P_OUTPUT_PLACE` = the PCB 1 mm above the floor).

**Alternatives.** The nominal pose from vacuum on: the PCB would jump 1.5 mm into the crate floor at the pick. The frozen
relative pose: above.

**Revisit if** the physics run shows the board hanging elsewhere in the cups (it reports the PCB's slip).

Pinned by `test/Program/test_core.py::test_the_carried_pcb_hangs_at_the_cup_face_once_the_bellows_extend`,
`::test_the_pcb_moves_through_the_cycle_by_the_rules`.

## PATH_FORK_OPEN keeps the recorded velocity step at its join

**Decision.** `Robot.MovePath(PATH_FORK_OPEN)` starts at its join row as recorded: the flange already moves at 39.1 mm/s there, and the MoveL
onto `P_MG06_OPEN_JOIN` ends at rest, so the trajectory has a velocity step at the join (owner, 2026-10-24). The build notes
it. The physics runs of both programs follow it: the opening's tracking error stays at 0.0016 rad (the run's maximum 0.0126
rad is in the closing) and the fork's force on the bar stays within the closing's (the physics baselines, 2026-10-25). *Corrected
2026-10-25: the known issue "PATH_FORK_OPEN starts with a velocity step" closed by these runs; the real robot's behaviour at the
join is part of "the recorded fork paths exceed the assumed acceleration limits" (KNOWN_ISSUES.md).*

**Why.** The verified physics ran the legacy plan with the same step; a time warp would change the recorded opening.

**Alternatives.** Ramping the first rows in with a time warp: smoother, but a change to the verified motion.

**Revisit if** a physics run shows a tracking error or a force on the bar at the join beyond the closing's.

Pinned by `test/Program/test_core.py::test_the_fork_paths_follow_the_recorded_rows_and_join_rule`.

## The program check holds the taught moves to the limits and reports the recorded paths

**Decision.** `Program.Check.Check_Trajectory` measures exact distances at every sample where anything may have moved more
than `Clearance.Check_Step` (1 mm) since the last measured one, and at every segment end; it fails a pair within the 3 mm
clearance unless the step allows its group, and an allowed pair deeper than the group's `Allowed_Depth`. It holds every MoveJ
and MoveL to the `Motion` limits, allowing only a MoveJ's rounding to whole 4 ms samples (the legacy rule rounds T: speeds up
to 1 + 0.5 / n, accelerations its square) and 0.1 % for finite differences; the recorded fork paths' peaks are reported, not
limited. MG06 moves, during a MoveJ / MoveL, from its state at the motion's start to the target's `Fixture` along the
motion's time law.

**Why.** Measured on the default program (2026-10-24): MoveJ peaks reach 1.0031 of the acceleration limit through the
rounding alone; the recorded paths reach 2.03x the assumed acceleration (`PATH_FORK_CLOSE`) and 8.44x at `PATH_FORK_OPEN`'s join
(the velocity step) - the verified physics ran them, so they are the reference, not a fault of the build. With MG06 switched
to the settled closed state at the start of the bar-centring MoveL the fork stood 5.95 mm in the bar; the handle really
settles as the fork lets go. Holds are skipped by the motion bound: 10 720 of the taught program's 17 602 samples are
measured (`test/Program/data/golden_check_report.txt`).

**Alternatives.** Every sample measured: the same result, about 1.6x the time. A fixed 1 % speed / acceleration tolerance:
would hide a MoveL 1 % too fast. The recorded paths held to the assumed limits: the verified motion would fail.

**Revisit if** the acceleration limits are measured on the real robot (KNOWN_ISSUES), or the fork paths are replaced.

Pinned by `test/Program/test_check.py::test_the_default_program_passes_its_check`,
`::test_a_movej_over_its_rule_limits_is_reported`, `::test_contacts_are_allowed_only_where_declared_and_as_deep_as_configured`,
`::test_a_colliding_step_is_reported_with_its_pair_and_time`, `::test_the_handover_keeps_the_board_off_the_ledges`,
`::test_a_press_at_the_handover_is_reported`.

## The default program lifts the PCB clear of the crate and returns from MG06 over the family switch

**Decision.** Two changes to the legacy sequence, found by the program check: the pick lifts the carried PCB to a new
auxiliary keypoint `P_INPUT_CLEAR` (170 mm above the PCB's place: the board's bottom 25 mm above the crate's 149 mm walls)
instead of `P_INPUT_ABOVE` (120 mm, the board still below the walls); and the return from MG06 is `P_MG06_ABOVE` →
`P_VIA_HOME_MG06` → `P_HOME`: the direct MoveJ drives the fork 8.41 mm through the handle bar, and the legacy planner had
inserted the same via on its return. *Corrected 2026-10-24: the first design returned straight home.*

**Why.** Measured: with the 120 mm lift the carried PCB passes 1.64 mm from the input crate's wall during the MoveJ to
`P_VIA_INPUT_LOADER` (the legacy plan's own motion; the legacy check, based on generated contacts, could not see it); with
`P_INPUT_CLEAR` nothing comes within 10 mm. Cost (measured 2026-10-24): +0.388 s for the lift, +0.6 s for the return over the
via.

**Alternatives.** Allowing `pcb_crate` in `S3_To_Loader`: would hide a real near-miss. A MoveL from `P_INPUT_ABOVE` to the via:
a diagonal climb out of the crate, not a straight lift.

**Revisit if** the crate is lower or the PCB smaller, or the owner wants the straight home move with another via.

Pinned by `test/Program/test_check.py::test_the_default_program_passes_its_check`,
`test/Keypoints/test_core.py::test_every_point_passes_the_check`.

## The CAD deviations stay compensated in scene.xml

**Decision.** `models/scene.xml` keeps three deliberate deviations from the CAD: the loader shifted
+100 mm in Y and +0.95 mm in X; the rear-right holder support (`holder_10`) lowered by 0.8 mm; the
electrical cabinet 200 mm deep instead of 300 mm. The scene is never regenerated from Blender.

**Why.** Without them the cycle does not work in the model (SPEC §4.10): the loader
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

**Decision.** The program of the cell is split by who uses each value (owner, 2026-10-24):
`Configuration/Constants.yaml` declares every named value the cell's controllers use (tools, points, loader positions,
gripper commands, recorded paths, I/O signals); `Configuration/Program.yaml` is what the controllers run (the robot's
segments `S<n>_<Name>`, the loader's and the tester's jobs `J_<NAME>`), one call per line; `Configuration/Simulation.yaml`
holds what only the simulation needs (the start state, the model site of each tool, the actuator of each output, the
response time of each input, where the jaws really stop, the settle time, the contacts per segment, each point's scene).
`Configuration/Cell.yaml` stays the cell's physical description. `src/Configuration/Program_Parser.py` is the only reader
of the three files and builds a typed intermediate representation (`Program_Str`) that everything else reads; the format
change kept every number (proved against a golden build, below).

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

**Alternatives.** Pose as the source with IK at load: the IK family and branch become a run-time choice. Lowercase names:
no kind in the name.

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
a person thinks in, and it keeps the offset valid when the place is re-taught (the names `Reference` / `Offset`: owner,
2026-10-24).

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

**Alternatives.** One mapping per command: verbose and far from a pendant program. A regular-expression parser: weaker
on nesting and quoting than `ast`, with no gain.

**Revisit if** blending or accelerations are implemented, or a real controller's syntax is adopted.

Pinned by `test/Configuration/test_program_parser.py::test_a_command_line_is_parsed_never_evaluated`,
`::test_the_yaml_loader_rejects_duplicate_keys_and_keeps_on_off_as_text`,
`::test_an_invalid_program_line_is_named_with_file_segment_line_and_text`, `::test_a_program_using_a_value_v_and_blank_a_r_loads`.

## Simulated I/O and the end settle live in Simulation.yaml

**Decision.** `Robot.SetDO(DO_VACUUM, ON)` drives the output's simulated actuator (`Simulation.yaml Actuators`), and
`Robot.WaitDI(DI_VACUUM_OK, timeout=1.0)` holds for the input's simulated response (`Signal_Response`, 0.3 s: the legacy
planner's hold after vacuum on) and fails the load when the response exceeds the timeout. The F8 program's last step
(a 0.8 s wait for the physics to settle) is `End_Settle` in `Simulation.yaml`, appended after the last segment: the taught
trajectory lasts 70.404 s, the program a controller would run 0.8 s less. The signals' `Port` is a placeholder
(`KNOWN_ISSUES.md`).

**Why.** A real program waits for its vacuum sensor, not for a time; the time it takes is a property of the simulated cell.
A settle wait for the physics run means nothing to a controller.

**Alternatives.** A plain `Wait(0.3)` after the vacuum: correct in time, but the real program would not check its vacuum.

**Revisit if** the real cell's I/O map is defined, or the vacuum model gets a pressure state.

Pinned by `test/Program/test_core.py::test_a_waitdi_holds_its_simulated_response_and_the_cycle_ends_with_the_settle`,
`test/Configuration/test_program_parser.py::test_signals_contacts_and_constants_are_complete`.

## Every constant is used; no field without effect

**Decision.** A constant declared in `Constants.yaml` and used by neither `Program.yaml` nor `Simulation.yaml` is a load
error, and so is a job no segment runs. So `T_FORK` is not declared (the fork's MoveLs and paths run on the flange,
`T_FLANGE`).

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

## The taught program's golden data is exact, captured by one tool

**Decision.** `test/Program/data/` holds the taught program's golden data: the sha256 of every trajectory array's bytes, the
cycle time, the events, the segments, each command's values as `float.hex`, the check report and the point table.
`Example/Program/golden_program.py` writes (`--out DIR --source TEXT`) and compares it; `test/Program/test_golden.py`
imports it. A change that moves a number on purpose re-captures the data in its own commit, listing what changed (its
provenance in the record's `source`). The format change to the three files was proved this way: every array byte-identical.

**Why.** A program or format change must not move a number unnoticed; byte identity is the strongest proof and cheap to
keep. One reader and writer of the format; a re-capture is one command.

**Alternatives.** Tolerance comparisons: would let a real change through. Comparing only the check report: blind to the
trajectory's internals. A name map beside old data: every change would extend a map of names that no longer differ.

**Revisit if** the trajectory format changes (the golden data is then re-captured in the new form; the old is in git).

Pinned by `test/Program/test_golden.py` and `test/Example/Program/test_golden_program.py`.

## The optimizer drops the multi-seed IK of a FREE segment's ends

**Decision.** The optimizer does not search several IK solutions for a FREE segment's start and end (owner, 2026-10-24). A FREE segment runs between the ends of LOCKED segments, whose stored joints are fixed; there is nothing for a
seed search to choose.

**Why.** A different IK solution at an end would change the LOCKED segment it joins, which must stay byte-identical.

**Alternatives.** Multi-seed IK with a free end pose: would re-teach process-critical points.

**Revisit if** a FREE segment ever ends at a point no LOCKED segment fixes.

Pinned by `test/Optimizer/test_measure.py::test_a_locked_part_is_byte_identical_from_its_start`.

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

**Why.** toppra (0.6.10, MIT; checked on PyPI 2026-10-24) has Linux x86_64 wheels only - no macOS or aarch64 wheel, no sdist
since 0.6.3 (a Cython build) - so it cannot be installed from a wheel on the primary platform. Collocation at the grid points alone left `u` unbounded where a
clamped spline's `q'` vanishes (its ends): the sampled acceleration reached 1.75x the limit between grid points and the
stretch made a 1.80 s transfer 3.11 s (measured 2026-10-24); with first-order interpolation the stretch is 0.1-0.3 %. The
interval in closed form, not by a vertex-enumeration LP (its open bounds of 1e9 lost precision). B3: the closed-form trapezoid
and triangle are met within 1.9e-7 (relative; 1000 intervals).

**Alternatives.** A jerk-limited timing: not approved (owner: bang-bang accelerations; the physics run reports the tracking).
toppra: not installable on the primary platform (above).

**Revisit if** the physics shows tracking errors from the acceleration steps, a real controller's limits differ, or toppra
ships a macOS arm64 wheel.

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
waits for each job and starts the next. When the build's check finds the robot within the clearance of the loader (I8), or
the spline comes into the volume of a job it does not wait for (the gating rule, F15), the spline moves after the start of the
next job - so after the WaitDone of the job it met; a meeting while the robot waits at its start, or one past the last job's
start, fails, naming the remedies. No spline is ever gated by a Wait (F15; until F15 the spline could also wait longer by the
meeting window, which put `Wait(1.648)` into the take). Shipped (`Data/Optimized/Program_Optimized.yaml`): To_MG06 runs its
spline during `J_INSERT`; To_Take goes to its waiting point `P_LOADER_WAIT` during `J_FETCH` and on to `P_LOADER_ABOVE` after
`Loader.WaitDone(J_RETURN_Y)`, during `J_RETURN_Z`. A job that comes where the robot has been cannot be gated by a later slot
(the robot would wait in its way): the transfer needs an exit point it reaches before the job starts (F16); To_MG06 goes to
`P_LOADER_WAIT` before `J_INSERT`.

**Why.** In the taught program the robot is at home while the loader fetches and returns; `P_LOADER_ABOVE` is in the
loader's travel height path. A later slot keeps the robot out of the loader's way whatever the loader's speed; a wait only
keeps it out at the loader's modelled speed (F14: 2.2 % slower broke I8).

**Alternatives.** Planning the robot against the loader's swept volume (the design's conservative variant): it would forbid
`P_LOADER_ABOVE` as an end point outright. A waiting point between: a second spline and a stop.

**Revisit if** the loader's positions or jobs change (the optimizer finds the slot again).

Pinned by `test/Optimizer/test_measure.py::test_the_interlocks_hold`, `test/Example/Program/test_optimize_program.py`.

## The robot waits for the loader's done, never for a time (F15)

**Decision.** `J_RETURN` is split into `J_RETURN_Y` (MoveZ L_TRAVEL, MoveY L_HOME) and `J_RETURN_Z` (MoveZ L_HOME, Wait 0.2), the
same motion and timing (owner, 2026-10-25). The optimized take runs `Loader.Start(J_RETURN_Y)`, `Loader.WaitDone(J_RETURN_Y)`,
`Loader.Start(J_RETURN_Z)`, `Robot.MoveSpline(W_TO_TAKE)`, `Loader.WaitDone(J_RETURN_Z)`; interlock I9 and the gating rule
(`Program.Check.Gating_Faults`, SPEC §7.11) are checked; the optimizer gates a spline only by a later slot, never by a Wait.

**Why.** F14 measured the old `Wait(1.648)`: J_RETURN 2.2 % slower broke I8, 7.6 % slower was a contact with the Z carriage.
With the loader's Y at home the whole `W_TO_TAKE` spline keeps more than 50 mm from the loader at any Z (`Report.Core.Clear_Height`),
so waiting for `J_RETURN_Y` is enough. Measured 2026-10-25: the cycle stays 58.716 s (the spline still ends before
`J_RETURN_Z` does); every loader job slowed up to 2 x keeps the robot at least 29.68 mm from the loader (I8 nominal 19.39 ->
29.68 mm kinematic, 19.40 -> 29.02 mm in physics, S11); the taught trajectory is byte-identical (only four `Loader.Run` labels
of the golden changed); the forced failure - the old timer schedule on the split jobs - breaks I9 and the gating rule while I8
still holds at the nominal timing (19.39 mm), which is why a distance check alone could not catch it.

**Choice inside.** The gating rule measures the robot against every pose the job sweeps (its swept volume), not the pose at the
same time. Until F16 a motion that started inside the volume and left it (To_MG06 from `P_LOADER_ABOVE` during `J_INSERT`, 1.52 s
ahead of the loader) was only reported; F16 made the rule symmetric (next entry).

**Alternatives.** A loader input "Y at home" (`DI_LOADER_Y_HOME`, a WaitDI): the same timing, but a new signal and a PLC change;
the split job uses the existing handshake. Keeping the wait with a margin (a longer Wait): still timing; it only moves the
failing slow-down.

**Revisit if** the loader's positions or job contents change (the clear height and the gating rule are measured again), or a
loader input is added (then it can gate instead of the split).

Pinned by `test/Program/test_check.py::test_the_optimized_program_keeps_the_gating_rule`,
`::test_a_take_gated_by_a_timer_breaks_the_gating_rule`, `test/Optimizer/test_measure.py::test_i9_fails_for_a_take_gated_by_a_timer`,
`test/Optimizer/test_core.py::test_a_spline_that_comes_into_a_running_job_s_volume_waits_for_the_job`,
`test/Report/test_core.py::test_a_slower_loader_only_delays_the_take`, the golden (`test/Program/test_golden.py`) and the physics
baselines.

## The robot leaves the loader's way before the job starts: an exit point, and a symmetric gating rule (F16)

**Decision.** The gating rule is symmetric (owner, 2026-10-25): while a parallel job may run, neither the robot (moving or
holding) nor the loader comes within `Loader_Free` of where the other is at any other time of the window ('robot enters',
'loader enters'); either is a fault. The robot is the master: To_MG06 runs `Robot.MoveSpline(W_TO_MG06_EXIT)` from
`P_LOADER_ABOVE` to `P_LOADER_WAIT` (`Optimization.yaml Exit_Points`), then `Loader.Start(J_INSERT)`, then
`Robot.MoveSpline(W_TO_MG06)` on to `P_MG06_FORK_START`; interlock I10 (I9's mirror) checks the order. The optimizer fails a
'loader enters' fault naming `Exit_Points` (a later slot would leave the robot in the job's way).

**Why.** `P_LOADER_ABOVE` lies 18.47 mm inside the volume J_INSERT's Z carriage sweeps at the travel height; with the job started at
once the robot left it only by starting at once (1.52 s ahead). Measured 2026-10-25 on that schedule (05556c4): a robot speed
override below 14.6 % broke I8, below 11.2 % the tool met the carriage (10 %: -3.92 mm) - and a first run on a real cell often
uses 10 %. With the split, `P_LOADER_WAIT` is 29.68 mm from all J_INSERT sweeps; no job breaks I8 at any loader slow-down up to
2 x or robot override down to 10 % (closest 29.68 mm); the gating rule finds no fault.

**Cost.** The cycle 58.716 -> 59.148 s (+0.432 s, +0.7 %; throughput 61.3 -> 60.9 PCB/h): the exit leg takes 0.432 s from stop to
stop and J_INSERT starts that much later, and the loader is on the critical path in S5. A trigger on the path's position
(J_INSERT started as the unsplit spline passes the volume's edge at 11.244 s, without a stop) would cost +0.244 s; the 0.188 s
between them is the stop at the split.

**Alternatives.** A path-position trigger in the spline (+0.244 s): a format change (a trigger on `Robot.MoveSpline`), the build
and the check of a mid-spline event, and a controller feature not verified on the AUBO. A split at the planned path's exit point:
an unnamed configuration (the format names its ends by taught points), not re-teachable. Starting J_INSERT with its Y move only
(splitting the job): the lift at home is in the robot's way too. Keeping the robot's prompt start: rests on timing.

**Revisit if** a trigger on a motion's progress becomes available and verified on the controller (then the 0.188 s can be
taken back), or the handover's exit is re-taught.

Pinned by `test/Program/test_check.py::test_the_optimized_program_keeps_the_gating_rule`,
`::test_an_insert_started_before_the_robot_leaves_breaks_the_gating_rule`,
`::test_a_robot_holding_in_a_running_job_s_way_breaks_the_gating_rule`,
`test/Optimizer/test_measure.py::test_i10_fails_for_an_insert_started_before_the_robot_leaves`,
`test/Optimizer/test_core.py::test_the_insert_starts_after_the_robot_leaves_for_its_exit_point`,
`test/Report/test_core.py::test_a_speed_override_breaks_only_an_ungated_insert` and the optimized physics baseline.

## The optimized program is generated data, committed and reproducible

**Decision.** `Data/Optimized/Program_Optimized.yaml` is written only by `Example/Program/optimize_program.py` and committed.
It records the sha256 of its inputs (the four configuration files, `Optimization.yaml`, the fork path, the compiled planning
model), the platform and versions, each segment's start time and the TOPP grid. A build refuses it when an input changed or
when it does not reproduce the schedule; the optimizer run again writes the same bytes (B5). A YAML input's hash - here and in
a trajectory's metadata - is of its loaded content (`Configuration.Core.YAML_Hash`: the document as compact JSON, each mapping
in the file's order), any other input's of its bytes (`Program.Core.Input_Hash`; owner, 2026-10-25).

**Why.** The planning is random with a fixed seed: the file keeps the result reviewable and diffable, and the hashes keep it
honest when the taught program or the cell changes. A comment is not an input: a comment-only edit of a hand-written file must
not make the optimized program or a trajectory stale (two Cell.yaml comment fixes each forced a regeneration, 2026-10-25).

**Alternatives.** Planning at every build: slow (the optimizer runs about 60 s) and the result unreviewed. Byte hashes of the
YAML files: every comment edit makes the outputs stale. Sorted keys in the content hash: a reordering would then not count,
although an order can matter to a reader.

**Revisit if** the optimizer runs on another platform (its output may then differ: re-run and commit with the platform).

Pinned by `test/Configuration/test_core.py::test_the_yaml_hash_is_the_content_not_the_bytes`,
`test/Program/test_core.py::test_an_input_is_hashed_by_its_yaml_content_or_its_bytes`,
`test/Example/Program/test_optimize_program.py::test_the_optimizer_reproduces_the_committed_program_byte_for_byte`,
`test/Optimizer/test_core.py::test_the_shipped_optimized_program_is_current_and_matches_its_schedule`,
`test/Example/Program/test_build_program.py::test_the_optimized_program_is_built_and_a_stale_or_edited_one_refused`.

## Run is Start plus WaitDone; a robot Wait holds only the robot

**Decision.** The robot's program is the master of the cycle. `Loader.Run(J_)` and `Tester.Run(J_)` run a station's job
and wait until it is done; the optimized program splits a loader job into `Loader.Start(J_)` and `Loader.WaitDone(J_)` (a
taught program refuses them), and `Tester.Start` / `Tester.WaitDone` are reserved (the tester's job runs whole). The builder
runs a started loader job one sample per robot sample (a motion, a `Wait`, a `WaitDI` of the robot all advance it);
`Loader.WaitDone` holds the robot until the job's last sample; `Loader.Run` gives the same samples as Start + WaitDone, so the
taught program's trajectory is unchanged (golden). A loader job runs one axis after the other (owner, 2026-10-24).
*Corrected 2026-10-25: merged with the entry "Loader.Run starts a job and waits; Start and WaitDone are reserved", which F9
superseded.*

**Why.** The real cell's loader is a PLC with its own program; a job keeps its steps in one place and the robot's program
says only when it starts and waits - and a real master program runs this way: the PLC's job goes on while the robot works.

**Alternatives.** Loader commands inline in the robot's program: the PLC's part could not be read as one unit. A separate
loader timeline merged afterwards: the PCB rules (clamp at MG06, release) would need the merge to know both lanes' states.

**Revisit if** the tester's job runs in parallel too (Tester.Start is reserved).

Pinned by `test/Program/test_core.py::test_a_run_is_a_start_and_a_waitdone`, `test/Optimizer/test_core.py::test_the_loader_moves_while_the_robot_moves`,
`test/Configuration/test_program_parser.py::test_an_invalid_program_line_is_named_with_file_segment_line_and_text`,
`::test_an_invalid_job_line_is_named`, `::test_the_commands_are_resolved`.

## The optimized program's longer PCB path is accepted

**Decision.** The optimized program carries the PCB farther than the taught one: at the decision (2026-10-24) 2.856 m against
2.712 m (+5 %, the 20 deg tilt limit), while the robot's joint travel fell 18 % and the TCP's path 13-15 %: accepted (owner,
2026-10-24). The current lengths: `Example/Program/compare_programs.py`.

**Why.** The optimizer's objective is time within the limits and the clearance; the FREE transfers that carry the PCB (To_Loader,
To_Output) are shorter in time, their clearances pass, and the tilt stays within its limit.

**Alternatives.** A path-length term in the planner: no gain in time, more tuning.

**Revisit if** the physics shows PCB slip in a carrying transfer above its warning.

Pinned by `test/Example/Program/test_compare_programs.py`.

## A robot stop is a tenth of a second at rest

**Decision.** The comparison counts a robot stop when every joint speed (its change per sample over 4 ms) stays below 1e-3 rad/s
for at least 0.1 s after a motion (`Optimizer.Measure.CONST_STOP_SPEED`, `CONST_STOP_TIME`), and reports the time at rest in
them. Both programs stop 12 times (`test/Example/Program/test_compare_programs.py`).

**Why.** A count without a duration (a sample moving followed by one at rest) counted the legacy plan's single still rows: 135
stops. With
a duration, the count is the process's holds - the vacuum at the pick, the handover's clamp, the fork paths' recorded rests,
the test, the take, the place - and the waits for the loader; the optimization removes motion and moves the waits, so the count
stays and the time at rest falls.

**Alternatives.** A speed threshold alone: still counts the transient zero speed of a min-jerk move's ends.

**Revisit if** the comparison should tell the process holds from the waits.

Pinned by `test/Optimizer/test_measure.py::test_a_stop_is_a_tenth_of_a_second_at_rest_after_a_motion`,
`test/Example/Program/test_compare_programs.py`.

## The optimized take waits at P_LOADER_WAIT; the loader is the critical path

**Decision.** In the optimized program the robot leaves `P_MG06_FORK_START` at once after Open_MG06 (with `J_FETCH`), waits at
`P_LOADER_WAIT` (50 mm beyond `P_LOADER_ABOVE` along the plate normal, `Optimization.yaml Waiting_Points`) and moves on to
`P_LOADER_ABOVE` once the loader's Y is home (`Loader.WaitDone(J_RETURN_Y)`, F15; until F15 a `Wait(1.648)` after `J_RETURN`
started). Interlock I7 stays: the take starts when `J_RETURN_Z` has ended; I9: the motion to the take after `J_RETURN_Y`.

**Why.** Owner's observation (2026-10-24): the robot waited 4.05 s above MG06. The loader is the critical path - the robot was
already at `P_LOADER_ABOVE` before the loader was home - so leaving earlier gains no time; it removes the robot's wait above
the tester (which a real cell's safety zones would have had to allow). `P_LOADER_ABOVE` itself cannot be the waiting point:
the `z_carriage` at its travel height meets the robot there. Other candidates (measured 2026-10-24): I7' (the take on the
loader at rest at home, during its trailing Wait(0.2)) would gain at most 0.2 s and needs a state-based "loader home" input
and a WaitDone inside the LOCKED take: not done; the wait before Close_MG06 is loader-limited by I3 (gain 0); starting
`J_INSERT` during the handover's last MoveL would gain at most 1.5 s, but it splits a LOCKED segment and relaxes I2 - an open
candidate (FEATURES.md), not done without the owner's decision, a time-dependent check and the physics proof. The waiting
point's distance: below (the free transfers keep 10 mm from the loader).

**Alternatives.** One spline straight to `P_LOADER_ABOVE` timed after the loader passes: the same time, the wait above MG06.

**Revisit if** the loader's jobs get faster (the robot may become the critical path), or I7' / I2' are wanted.

Pinned by `test/Optimizer/test_core.py::test_the_robot_leaves_mg06_at_once_and_waits_at_p_loader_wait`,
`test/Optimizer/test_measure.py::test_the_interlocks_hold`,
`test/Configuration/test_program_parser.py::test_an_optimizer_point_is_used_only_by_the_optimized_program`.

## The take relieves the press before the jaws open

**Decision.** `S18_Take_From_Loader`, after the vacuum is OK, moves 1.5 mm back along the tool axis to `P_LOADER_TAKE_TOUCH`
(Reference `P_LOADER_PICK`, Offset [0, 0, -0.0015], at 10 mm/s), then opens the jaws, then waits for `DI_VACUUM_OK` again
(the board still held after the release; it costs nothing when the input is already true). The segment stays LOCKED and the
same in both programs; `P_LOADER_PICK` stays as taught (owner, 2026-10-24). Cost: +0.544 s per cycle (the touch move 0.600 s,
the exit 0.056 s shorter).

**Why.** In physics the compressed bellows' 10.8 N throws the board off the cups when the jaws let go (KNOWN_ISSUES.md);
relieved first, both programs hold 4 of 4 cups from the release through the carry in every sensitivity run (start -50 to +50
µm, ±0.2 deg).

**Alternatives.** Re-teaching the pick to the board's measured pose in the jaws (0.74 deg, -0.16 mm) with the 1.5 mm press:
the press even, the board still lost (58.430 s), and the kinematic check, posing the nominal board, read the press 2.16 mm
deep. Reusing `P_LOADER_PLACE` (the same pose within 6 µm): the handover and the take are different operations, and the
handover's no-press check is keyed on that point. Opening the jaws slower: untested, the stored force stays.

**Revisit if** the vacuum model gets a holding range, or the real cell shows the take holds with the press.

Pinned by `test/Program/test_core.py::test_a_waitdi_holds_its_simulated_response_and_the_cycle_ends_with_the_settle`, the
golden data (`test/Program/test_golden.py`) and the physics acceptance of both programs.

## A WaitDI waits only for what is left of its input's response

**Decision.** A simulated input comes its `Signal_Response` after its cause (`Program.Core.CONST_INPUT_CAUSE`: `DI_VACUUM_OK`
after `DO_VACUUM` went on); a WaitDI holds what is left of that time, nothing when it has passed; with the cause off the
build fails (the wait would time out).

**Why.** A real WaitDI returns at once when its input is true; the take's second WaitDI (after the jaws open) must not add
0.3 s to the cycle.

**Alternatives.** Each WaitDI holds the full response: adds a wait the cell would not have.

**Revisit if** an input gets a cause other than an output, or more inputs exist.

Pinned by `test/Program/test_core.py::test_a_waitdi_holds_only_what_is_left_of_the_response` and
`::test_a_broken_world_rule_is_refused`.

## The simulated vacuum switch counts cups in physics only

**Decision.** `DI_VACUUM_OK` is a sensor model in the physics run: true once `Simulation.yaml Vacuum_OK_Min_Cups` (4) cups have
touched the PCB with the vacuum on for its `Signal_Response` (0.3 s); every `WaitDI(DI_VACUUM_OK)` is checked where its wait
ends on the trajectory's timeline, and a false input stops the run with the step, the line, the cups and how long they held
(`vacuum not OK at ...`, exit 1). The program's acceptance has its items for it (SPEC §10.1 items 8-10).

**Why.** The owner's rule: the program waits for a real vacuum signal; the post-release WaitDI then means something in physics.
The kinematic build does not count cups: it attaches the PCB to the cups at vacuum on by its rules (the held pose within 2 mm,
`CONST_PCB_TOLERANCE`) and poses the board in the jaws at the nominal hold pose; a cup count there would always be 4 and prove
nothing.

**Alternatives.** Letting a WaitDI extend the timeline until the input comes: physics follows the trajectory's fixed timeline;
an input later than the build expected is reported as the wait failing. Counting cups in the kinematic build: always 4 (above).

**Revisit if** the trajectory gets event-driven waits, or the vacuum model gets a pressure state.

Pinned by `test/Simulation/test_core.py::test_the_vacuum_switch_fails_a_waitdi_whose_input_has_not_come`,
`::test_the_old_take_loses_the_board_at_the_release` and `test/Simulation/test_report.py::test_a_program_run_fails_exactly_its_new_criterion`.

## The free transfers keep 10 mm from the loader where its actuators put it

**Decision.** In every free transfer of both programs (the taught program's `Free_Segments`, the optimized program's splines)
the robot and the carried PCB keep `Cell.yaml Clearance.Loader_Free` (10 mm, owner) plus `Tracking_Margin` (4 mm) from the
loader's carriages and jaws and the PCB in them, at every sample, with the loader posed by `Program.Check.Loader_Actual` (the
model's actuator: filter, servo, axis mass, damping and gravity load) instead of its command. The planner's validity has the
same rule against the static scene, `Delay` and I8 against the moving loader. `P_LOADER_WAIT` moved from 25 to 50 mm above
`P_LOADER_ABOVE` along the tool axis. The taught vias stay: the taught transfers already keep 20.6 mm (S3_To_Loader) and more
than 50 mm (the others).

**Why.** Physics showed the optimized take's robot 0.51 mm from the z carriage where the kinematic check, with the loader at its
command, measured 4.28 mm: the command leads the real loader by up to 22.8 mm. The actuator model reproduces physics within
0.17 mm (y) and 1.04 mm (z); a first-order filter alone was 9.6 mm off. The margin covers the robot's largest deviation from its
reference in the free transfers (2.51 mm) and the model's error (1.04 mm), rounded up.

**Measured (2026-10-25).** The waiting point's distance to the loader's swept volume (J_FETCH + J_RETURN): 25 mm -> 5.32 mm,
30 -> 10.04, 35 -> 14.90, 40 -> 19.80, 45 -> 24.72, 50 -> 29.64, 60 -> 39.50 mm: 50 mm is the nearest that keeps the
10 + 4 mm with a margin. The rule cost the optimized cycle 0.056 s.

**Alternatives.** The command as the loader's position plus a larger margin: 22.8 mm of lag would make it ~25 mm everywhere.
A first-order lag (the owner's first reading of the 30 ms): 9.6 mm off. Raising the taught vias: not needed.

**Revisit if** the loader's masses or servo gains change (the model reads them from the compiled cell), or the real loader's
response is measured.

Pinned by `test/Program/test_check.py::test_the_free_transfers_keep_the_loader_clearance`,
`::test_the_loader_actuator_model_lags_and_settles`, `::test_the_loader_actuator_model_matches_the_physics` (slow),
`test/Optimizer/test_core.py::test_the_free_space_keeps_the_clearance_and_the_tilt` and
`::test_the_robot_leaves_mg06_at_once_and_waits_at_p_loader_wait`.

## The recorded closing path stays, and the fork is bounded on its net force and hinge load

**Decision.** `PATH_FORK_CLOSE` is played as recorded (it presses up to 0.54 mm into the handle bar). The fork's bounds use the
net contact force on the bar and the handle hinge load (SPEC §10.1); the summed normal force is reported for information.

**Why.** The force is chaotic (4 or 6 contact points, flipped by sub-micrometre differences, with different values; KNOWN_ISSUES.md). A retraction
of the recording off the bar (a `Bar_Retraction` per path, the rows moved along every nearby fork face's normal, fading out at a
0.2 mm gap; experiment only, kept off main) was tested with 14 runs each (start -5 / -1 / 0 / +1 / +5 µm,
waits 0 / 4 ms / 0.1 s, both programs), 2026-10-25:

| Retraction | Overlap left | Latch (handle / lid) | Modes | Net force | Hinge load | Summed (info) | Acceptance |
|---|---|---|---|---|---|---|---|
| 0 (as recorded) | 0.539 mm | 28.65 / -0.48 deg | 2 | 229.0-247.3 N | 9.38-11.45 N m | 291-371 N | 14/14 |
| 0.55 mm | none | 28.53 / -0.49 deg | 1 | 186.8 N | 8.23 N m | 250 N | 14/14 |
| 0.60 mm | none | 28.51 / -0.49 deg | 1 | 201.2 N | 9.23 N m | 318 N | 14/14 |
| 0.65 mm | none | 25.51 / -0.64 deg | 1 | 406.7 N | 9.14 N m | 268 N | 13/14 (no latch) |
| 0.70 mm | none | 25.51 / -0.64 deg | 1 | 406.7 N | 8.66 N m | 262 N | 13/14 (no latch) |

The owner's rule adopts a retraction only when 0.55-0.65 mm all pass; 0.65 mm does not latch MG06, so the passing window
(0.55-0.60 mm) is too narrow and the recording stays. The criterion is the net force, not the summed force: the summed force
adds the squeeze between the slot's walls.

**How to repeat it:** `Data/Experiments/fork_bar_retraction/README.md` (its patch against `92d1943`, the sweep script and the
runs; the experiment's own commit is not on main).

**Bounds** (from the worst mode x 1.15): net force 247.3 N -> 290 N; hinge load 11.45 N m -> 13.5 N m (rounded up to 10 N and
0.5 N m); the summed force (371 N x 1.15 -> 430 N) is reported, not bounded.

**Alternatives.** Adopting 0.55-0.60 mm: unimodal and lower, but 0.05 mm from a path that no longer latches. A re-recorded
path: outside this project's data. A settle wait before the path: not the cause (any wait flips the mode).

These bounds are regression bounds derived from the simulation, not safety limits (the vendors' ratings are not known;
KNOWN_ISSUES.md).

**Revisit if** the real MG06's handle force limit or the VGC10's force and moment ratings are known, the closing on the real
cell shows a problem (a force too high, a collision stop, wear of the bar; the real fork and MG06 would need their own sweep;
owner, 2026-10-25: not the narrow window now), or the path is re-recorded.

Pinned by `test/Simulation/test_core.py::test_the_fork_force_is_reported_as_net_squeeze_and_hinge_load` and the step 6 bounds.

## The legacy cycle run is retired; the programs are the regression

**Decision.** `Example/Simulation/run_cycle.py`, `Data/Reference/cell_plan.npz`, `test/Simulation/data/baseline_report.txt`,
`Simulation.Legacy_Drive_Cls` / `Run_Cycle` / `Read_Legacy_Plan` and their tests are removed (owner, 2026-10-25); the last
commit with them is `7a8560c8c16806739a02da8e989c2a2f65d1e41d`; the legacy planner and cycle test are in
`f754e2d0379b06ffb43286f4fb7aabf88e25ca5f` (referenced by commit hash: tags are for releases only). Two known issues of the legacy
run closed with it: its crate penetration of -0.14 mm against the brief's -0.08 mm, and its take that carried the board on 2 of
4 cups. *Corrected 2026-10-25: merged with the entry "The legacy plan stays the regression input until step 6", which this
superseded.*

**Why.** The keypoint programs replace the legacy plan as the regression (SPEC §2 item 6); the legacy run proved the port of
the physics loop and its take was sound by chance only.

**Alternatives.** Keeping it as a second regression: a knife-edge run that a 10 µm change fails, kept only for history.

**Revisit if** the physics loop is rewritten and needs the old proof again (check out `7a8560c8c16806739a02da8e989c2a2f65d1e41d`).

Pinned by `test/Simulation/test_baselines.py`.

## The physics regression checks bounds and drift, not the report's text

**Decision.** The programs' physics baselines fail a run on SPEC §10.1 and on metrics drifting beyond stated tolerances
(SPEC §10.2: the cycle time exact, events +-1 sample, PCB poses +-0.05 mm, the slip +-0.2 mm, forces and tracking +-10 %, ...);
the full report is kept as the baseline and its diff is printed for information only. The kinematic golden data and the model
fingerprint stay exact.

**Why.** Contact physics is sensitive to sub-micrometre changes: the fork's force flips between two modes with a 1 µm change
in the PCB's start (F11), the legacy take held the board on 2 cups or lost it with 10 µm; an exact report diff would fail on
any platform, solver or compiler difference that changes nothing physical, and train people to re-capture blindly.

**Alternatives.** The exact diff of the legacy baseline: right for a refactor proof, wrong as a regression of contact physics.
Tolerances wide enough for both fork modes: the hinge load differs by 22 % between them, which would hide a real change of
either mode.

**The fork's envelope** (owner, 2026-10-25): the fork's net force, the
hinge load and the J2 / J3 torques pass inside their program's envelope ± 10 % - min and max over 10 perturbation runs (the
PCB -5 / -1 / 0 / +1 / +5 µm without and -5 / 0 / +5 µm with a 0.1 s settle wait, the handle hinge ±1 µrad; ±1 µm added to
the owner's 8: they give the taught program's 247.3 N / 11.45 N m outcome, which the 8 missed) - and always within §10.1. Why: the contact
is chaotic and its outcomes are not well-defined modes - the contact count does not name one (6 contact points gave 247.3 N /
11.45 N m in the taught program and 222.2 N / 10.01 N m in the optimized one, a handle offset of -1 µrad; the optimized
program never reached 6 points from a PCB offset). Alternatives: modes keyed by the contact count (they failed on a 6-point
outcome with other values) or clusters of values (no fixed number of clusters to rely on). The contact count is reported only.

**Revisit if** a platform change makes a tolerance trip with no physical change, or a metric needs a tighter tolerance.

Pinned by `test/Simulation/test_baselines.py` (the envelope: inside, outside on both sides, J1 against the run, a failed bound
inside the envelope) and
`test/Example/Simulation/test_physics_baseline.py`.

## A program's physics runs through one loop fed by a trajectory drive

**Decision (owner, 2026-10-24).** The physics run of a program (`Simulation.Physics_Run_Cls`, `Run_Physics`) is the loop that
was the legacy cycle run's (servo set-points with velocity feed-forward, the filtered loader drives, vacuum adhesion, the
instability guard, every measurement), fed by `Trajectory_Drive_Cls` from a program's trajectory file. What the legacy run
took from its phase labels comes from the program: the fork's allowed contact from the segments' `Allow_Contact`, the
"placed" and "closed" marks from `J_PARK` and the `Test` step.

**Why.** One loop keeps one physics: the programs are measured as the verified plan was (the port proved by the legacy
report, byte-identical, until the legacy run was retired).

**Alternatives.** A second physics loop for trajectories: two loops to keep equal. Converting a trajectory into the legacy plan
format: loses the analytic speeds and the steps.

**Revisit if** a program needs a mark or a contact rule the trajectory does not carry.

Pinned by `test/Simulation/test_core.py` (the drive's interpolation and step classes), `test/Simulation/test_baselines.py` and
`test/Example/Simulation/test_run_program.py`.

## The carried PCB tilts at most 20 deg

**Decision.** `Optimization.yaml Carry_Max_Tilt` is 20 deg (owner, 2026-10-24), not the first, assumed 30 deg.

**Why.** It costs no time (at the decision, 2026-10-24: To_Output at 17.73 deg in 1.768 s against 28.98 deg in 1.792 s at
30 deg; the cycle 24 ms shorter); a flatter board is less likely to slip on the cups (KNOWN_ISSUES.md).

**Alternatives.** 30 deg: the same time, a board tilted almost 29 deg while it accelerates.

**Confirmed** by the physics run (2026-10-25): the carries slip on 4 of 4 cups below the 1.98 mm warning (KNOWN_ISSUES.md); the
owner keeps 20 deg.

**Revisit if** a real-cell value exists, or a later program's physics run warns.

Pinned by `test/Optimizer/test_measure.py::test_the_carried_pcb_keeps_its_tilt_limit`.

## The loader gripper's jaws are synchronized and grip with 70 N

**Decision.** `models/scene.xml`'s `jaw_sync` equality is stiff (solref 0.001 - twice the 0.5 ms timestep, the stiffest MuJoCo
honours by default - and solimp 0.99 / 0.999, as the PCB contacts) and the gripper's force limit is 70 N, the sum of both jaws
(owner, 2026-10-25).

**Why.** The SCHUNK EGI 040-EI (ID 1474382, data sheet) synchronizes its jaws by a toothed belt and grips with 25-70 N as the sum
of both jaws. The former soft coupling (solref 0.004) gave 0.74 mm under the 80 N clamp, so the PCB was carried 0.37 mm off-centre,
landed on MG06's side guide holder_17 (0.18 mm clearance), was pushed 3.5 mm down along it and dropped 1.4 mm at the release;
80 N (40 N per jaw) was above the EGI 040's maximum. Now the jaws differ by 0.007 mm under load. It also resolves the
board's sag in the jaws after the re-grip (0.74-0.84 deg with the soft jaws, F11): 0.01 deg.

**Alternatives.** A tendon on both jaws: splits the force but does not synchronize the positions. The soft coupling kept: the
guide landing and the drop. 80 N: above the data sheet. The real cell's force setting is not known (KNOWN_ISSUES.md); 25 and
45 N give the same results (28/28, 4/4 cups, the PCB at most 0.007 mm in the jaws while the loader moves; 2026-10-25).

**Revisit if** the real gripper's force setting or centring is measured.

Pinned by the fingerprint (three entries), the place acceptance items (SPEC §10.1 18-25) and the physics baselines.

## The gripper ramps in 0.6 s: the EGI's jaw speed

**Decision.** Every `Gripper(G_, t=)` in the loader jobs takes 0.6 s; the program check bounds every ramp's peak by
`Motion.Jaw_Speed_Max` (0.1 m/s, the EGI 040-EI's max. speed).

**Why.** A min-jerk ramp peaks at 1.875 x its mean speed: the full 29.25 mm per jaw needs at least 0.548 s; at 0.55 s the
physics jaws still reached 100.0 mm/s (the servo's overshoot), at 0.6 s 91.4 mm/s commanded, 92.0 mm/s in physics. The legacy
0.35 s peaked at 157 mm/s. Cost: +1.008 s per program (accepted by the owner); the slower clamp also removed a 2-cup moment at
the handover clamp and lowered the jaws' penetration into the board (-0.108 -> -0.062 mm).

**Alternatives.** 0.55 s: the physics jaws reach 100.0 mm/s, no margin. The legacy 0.35 s: 157 mm/s, above the data sheet.

**Revisit if** the real gripper's speed setting is known.

Pinned by `test/Program/test_check.py::test_the_gripper_ramps_within_the_datasheet_jaw_speed` and its forced failure.

## The handover is taught 0.4 mm along the jaw axis

**Decision.** `P_LOADER_PLACE`'s `PCB_Hold` offset is [0.0004, 0, 0] (its tool x is the jaw axis); the six points derived from
it follow. The program check bounds the kinematic PCB's offset from the jaw centreline (`Handover.Lateral_Max` 0.5 mm) and
measures the descent by the PCB's height above the ledges.

**Why.** The jaws centre the PCB as they clamp. In physics the PCB sits on the cups 1.226 mm off the TCP after the pick and slips
0.41 / 0.46 mm by the handover, 0.372 / 0.425 mm short of the jaw centreline (taught / optimized); with the synchronized jaws the
clamp pulled it that far across the cups. One shared point leaves -0.026 / +0.026 mm of centring. Margin (the PCB moved on the
cups after the pick): along the jaw axis +-0.5 mm still keeps 4/4 cups (centring 0.47-0.52 mm); across it (tool y)
nothing is centred - +0.5 mm gives a PCB-in-MG06 error of 0.64 / 0.76 mm and 0.001 mm side-guide clearance (KNOWN_ISSUES.md).

**Alternatives.** Teaching the kinematic model the measured cup offset (moves every PCB-hold point); keeping the centred point
(the clamp pulls the PCB across the cups).

**Revisit if** the real handover is taught with real boards.

Pinned by `test/Program/test_core.py` (the handover tests), the point table and the physics baselines (`handover_centring`).

## The loader releases the PCB 0.15 mm above its MG06 seat; the re-grip has its own height

**Decision.** `L_PLACE` is 0.37 mm above the CAD place height; `J_FETCH` goes to its own `L_FETCH` (+0.035 mm). Release, never
press; re-grip with the PCB 0.3 +- 0.1 mm above the finger ledges.

**Why.** With the PCB centred, `L_PLACE` pressed it 9 N into the supports (the centred PCB touches the seat 0.216 / 0.223 mm above
it in physics; statically 0.124 mm above `place_pos`, the PCB 0.09 mm lower in the jaws). +0.37 mm releases it 0.152 / 0.151 mm
above the seat: it drops and is seated on holder_03 / 04 / 10 in 0.014 s, 0.040 / 0.037 mm transient penetration. At the raised
height the re-grip closed the jaws with the ledges 0.066 mm above the board's underside - on the ledge corners - so the fetch got
`L_FETCH`: the kinematic gap 0.265 mm (the PCB at the CAD place pose, 0.10-0.12 mm into the supports), 0.359 mm in physics.

**Alternatives.** The static seat alone (+0.274 mm): 0.06 mm of release height in physics. One height for both: on the ledges.

**Revisit if** the real loader's release and re-grip heights are measured.

Pinned by the place acceptance items, the re-grip check and the physics baselines.

## Jaw_Actual is the physics grip, and the posed jaws are the command limited by the part

**Decision.** `Simulation.yaml Jaw_Actual.G_CLAMP` is 17.54 mm (the synchronized jaws' grip in physics, 17.537-17.550 mm); the
kinematic posing shows the jaws at the command, never closer than grip while the PCB is between them - in the jaws, on the cups
at the handover or the take, or resting in MG06 (`Jaw_Posing_Faults` in the program check).

**Why.** 17.75 mm was the soft coupling's grip (0.20 mm off the board's edges). Posing the jaws at the command alone showed
them passing through the PCB at the release in MG06, which physics never did; the collision check does not test the PCB in
MG06, so it could not catch that.

**Alternatives.** 17.75 mm, the soft coupling's grip: 0.2 mm off the synchronized jaws. The jaws posed at the command: through
the PCB.

**Revisit if** the real grip is measured.

Pinned by `test/Program/test_check.py::test_the_posed_jaws_are_the_command_limited_by_the_part` and its forced failure.

## The place into MG06 is accepted on eight physics items

**Decision.** The place into MG06 is accepted on SPEC §10.1 items 18-25 (owner, 2026-10-25), with the bounds in `Cell.yaml
Acceptance` and `Handover`.

**Why.** The owner's decisions. The transient bound is 0.10 mm, not 0.05 mm: the margin runs reach 0.068 mm with the PCB 0.5 mm
off the cups' centre; it still catches the old pressed place (0.157 / 0.181 mm). While the optimized program's robot moves in
parallel (S5_To_MG06) it stays more than 200 mm from the loader and the PCB during the place and the release, and more than 50 mm
from the loader over the whole step.

**Alternatives.** A transient bound of 0.05 mm: the margin runs fail it.

**Revisit if** the real place is measured.

Pinned by `test/Simulation/test_report.py::test_a_program_run_fails_exactly_its_new_criterion` and the physics baselines.

## The module layout: Core.py, a topic module per second concern, Const.py only for shared constants

**Decision.** Each `src/` domain has `Core.py`, its primary module. A domain with a second concern puts it in a topic
module named for it, which imports `Core.py`, never the other way round: `Configuration/Program_Parser.py`,
`Program/Check.py`, `Simulation/Report.py`, `Optimizer/Measure.py`. A constant one module uses is defined in that
module; a constant more than one module of the domain uses is defined in the domain's `Const.py` (today Configuration,
Program, Simulation), which holds only constants. Tests mirror the modules (`test/<Domain>/test_<module>.py`; a
`Const.py` is tested through its readers). The owner's rule, 2026-10-25.

**Why.** The four largest modules (Program 1695, Program_Parser 1690, Simulation 1312, Optimizer 1103 lines) each held
two concerns; split by topic, a module reads as one concern and the import shows which one a caller needs. A constant
next to its only reader keeps the value and its why-comment beside the code they explain.

**Alternatives.** Core.py / Utilities.py / Const.py in every domain, by kind: rejected - whether a function is a
"helper" is not a property of the code (all of Collision's functions would qualify, none of Simulation's), constants
would leave their only readers, and the large files would stay large. One Core.py per domain (the skill's default):
kept for the seven domains with one concern.

**Revisit if** a topic module grows a second concern of its own, or a domain's Const.py would hold a constant only one
module uses.

This departs from project-style-rpx where it says each `*_Str` lives in its domain's `Core.py`: a topic module's own
structures live with it (`Program_Check_Str` in `Program/Check.py`). Pinned by `test/Docs/test_layout.py`
(`test_each_domain_holds_its_declared_modules`, `test_a_constant_lives_with_its_only_reader_or_in_the_domains_const`).

## The physics run's phase sampling and cup-touch threshold stay at 20 steps and 0.5 mm

**Decision.** `Cell.yaml Simulation.Phase_Steps` stays 20 (the step label sampled every 10 ms) and `Cup_Touch` 0.5 mm (a cup
counts as touching below that contact distance), the legacy run's values (owner, 2026-10-25).

**Why.** Measured on both programs (2026-10-25, the committed baselines' trajectories, the values changed in memory only):

| Variant | Taught | Optimized |
|---|---|---|
| `Cup_Touch` 0.1 mm | report and every metric byte-identical | byte-identical |
| `Phase_Steps` 10 | no metric, mark or acceptance item moves (28/28, 0 regression failures); the per-step table's boundaries move by 5 ms (20 rows) | the same (14 rows); three per-step tracking values by 0.0001 rad (start 0.0001 -> 0.0000, S2_Pick_Input 0.0012 -> 0.0013, S14_Place_Output 0.0009 -> 0.0010 rad), the run's maximum unchanged (0.0126 rad) |
| both | the same as `Phase_Steps` 10 | the same as `Phase_Steps` 10 |

The cups either touch the PCB (contact distance about 0) or stand well clear of it, so the threshold between 0.1 and 0.5 mm
decides nothing; the phase sampling only quantizes the per-step table's boundaries.

**Alternatives.** `Phase_Steps` 10: 5 ms boundaries in the per-step table, twice the label lookups, no other effect.

**Revisit if** a run's cup count or a mark ever depends on these values (a vacuum with a holding range, a shorter step).

Pinned by the physics baselines (`test/Simulation/test_baselines.py`), which hold the current values.
