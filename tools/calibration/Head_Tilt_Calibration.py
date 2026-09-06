import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from Soccer.config_paths import repo_root_str
from Soccer.Localisation.class_Glob import Glob
from Soccer.Vision.class_Vision_RPI import Vision_RPI
from Soccer.Motion.class_Motion_real import Motion_real as Motion


SIMULATION = 5
current_work_directory = repo_root_str(REPO_ROOT)


glob = Glob(SIMULATION, current_work_directory, particles_number=100)
vision = Vision_RPI(glob)
motion = Motion(glob, vision)
motion.activation()
motion.head_Tilt_Calibration()
