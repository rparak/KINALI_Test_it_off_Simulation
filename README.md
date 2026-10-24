# KINALI Test it off – cell simulation (MuJoCo)

MuJoCo simulation of the KINALI "Test it off" automated PCBA test cell: an AUBO i5 robot with an OnRobot VGC10 vacuum
gripper and fork picks a PCB from the input crate and hands it to a two-axis loader, which inserts it into the MG06 test
fixture. The robot closes and opens MG06, and the PCB goes back to the output crate. The physics is verified against
a regression baseline. A keypoint-based robot program is in progress. A full README follows in Phase 1 step 7.

> [!WARNING]
> **Outputs of this project (keypoints, programs, trajectories) are simulation results and are NOT validated for driving the real cell.** Before any use on real hardware they must go through the real-cell validation procedure (to be defined), with reduced speed and the robot controller's own safety limits active.

```bash
./install.sh                                                    # install: .venv with Python 3.11 and the pinned requirements
.venv/bin/python Example/run_cycle.py                           # headless physics run of the cycle + report (output/report.txt)
.venv/bin/python Example/export_cell_scene.py                   # write models/cell_scene.xml
.venv/bin/python -m mujoco.viewer --mjcf=models/cell_scene.xml  # interactive viewer
```
