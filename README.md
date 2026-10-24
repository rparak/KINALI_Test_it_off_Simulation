# KINALI Test it off – cell simulation (MuJoCo)

MuJoCo simulation of the KINALI "Test it off" automated PCBA test cell: an AUBO i5 robot with an OnRobot VGC10 vacuum
gripper and fork picks a PCB from the input crate and hands it to a two-axis loader, which inserts it into the MG06 test
fixture. The robot closes and opens MG06, and the PCB goes back to the output crate. The physics is verified against
a regression baseline. A keypoint-based robot program is in progress. A full README follows in Phase 1 step 7. The
previous Czech description of the cell is in [docs/README_cs_mg06_cell.md](docs/README_cs_mg06_cell.md).

```bash
python3 -m venv .venv && .venv/bin/pip install -e ".[dev]"     # install (MuJoCo 3.16.0)
.venv/bin/kinali-sim run                                        # headless physics run of the cycle + report (output/report.txt)
.venv/bin/kinali-sim export-xml                                 # write models/cell_scene.xml
.venv/bin/python -m mujoco.viewer --mjcf=models/cell_scene.xml  # interactive viewer
```
