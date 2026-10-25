# Test-it-off Simulation – Automated PCBA Testing Cell in MuJoCo

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="Media/JIC_Smart_Factory_White.svg">
    <img src="Media/JIC_Smart_Factory_Black.svg" alt="The JIC Smart Factory logo: the J, I and C bars and the words Smart Factory, in black or white" height="40">
  </picture>
  &nbsp;&nbsp;&nbsp;&nbsp;
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="Media/KINALI_White.svg">
    <img src="Media/KINALI_Black.svg" alt="The KINALI logo: the word KINALI between three dots, in black or white for the page theme" height="40">
  </picture>
</p>

Test-it-off Simulation is a MuJoCo simulation of KINALI's Test-it-off automated PCBA test cell: an AUBO i5 robot with an OnRobot VGC10 vacuum
gripper and fork picks a PCB from the input crate and hands it to a two-axis loader, which inserts it into the MG06 test
fixture. The robot closes and opens MG06, and the PCB goes back to the output crate. The physics is verified against
a regression baseline. A default robot program, taught from named robot points, is built into a trajectory and checked
kinematically (clearance, limits). An optimized version plans its free-space moves and runs the loader in parallel, with
the same checks (58.7 s cycle against 70.4 s in the simulation). Both programs run with full physics and meet the acceptance
criteria there (the PCB held on all four cups, never lost, room around the loader). A full README follows in step 7.

An internal research project of JIC Smart Factory, part of its internal research supporting small and medium-sized enterprises in advanced simulation. Developed in collaboration with KINALI for the development of the Test-it-off system, an automated workcell for testing assembled printed circuit boards (PCBA).

> [!WARNING]
> **Outputs of this project (keypoints, programs, trajectories) are simulation results and are NOT validated for driving the real cell.** Before any use on real hardware they must go through the real-cell validation procedure (to be defined), with reduced speed and the robot controller's own safety limits active.

```bash
./install.sh                                                    # install: .venv with Python 3.11 and the pinned requirements
.venv/bin/python Example/Scene/export_cell_scene.py                   # write models/cell_scene.xml
.venv/bin/python -m mujoco.viewer --mjcf=models/cell_scene.xml  # interactive viewer
.venv/bin/python Example/Program/build_program.py               # default program -> trajectory + check report (Data/Generated/)
.venv/bin/python Example/Program/show_program.py --step S11_Test --headless   # the cell at the end of a program segment
.venv/bin/mjpython Example/Program/show_program.py             # play the program in the viewer (macOS: mjpython)
.venv/bin/python Example/Program/optimize_program.py            # optimized program -> Data/Optimized/Program_Optimized.yaml + report
.venv/bin/python Example/Program/build_program.py --program Data/Optimized/Program_Optimized.yaml   # its trajectory + check
.venv/bin/mjpython Example/Program/show_program.py --version optimized   # play the optimized program
.venv/bin/python Example/Program/compare_programs.py            # taught vs optimized: table + figure (Data/Generated/)
.venv/bin/python Example/Simulation/run_program.py --program taught       # physics run of a program + report (Data/Generated/Reports/)
.venv/bin/mjpython Example/Simulation/show_physics.py --program optimized   # its physics in the viewer, real time (--speed 0.25: slower)
```
