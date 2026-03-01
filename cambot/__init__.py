"""Cambot — StereoBot teleop system."""

from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
CALIBRATION_DIR = PROJECT_ROOT / "calibration"
URDF_PATH = PROJECT_ROOT / "urdf" / "stereobot.urdf"
