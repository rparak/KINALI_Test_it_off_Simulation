"""Legacy shim: the names of the former robot_kin.py for legacy/plan_cell.py (F3.3: still the brief-layout
module src/kinali_sim/kinematics.py; F3.4 maps them onto Kinematics.Core and Collision.Core)."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))
from kinali_sim.kinematics import *   # noqa: F401,F403
