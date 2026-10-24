import os
import sys
# pyproject.toml is gone (FEATURES.md F3.1): reach the brief-layout package through src/ until F3.6.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
