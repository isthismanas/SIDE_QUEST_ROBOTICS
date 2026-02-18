#!/bin/bash
set -e

# Always run from repo root (two levels up from motion/)
cd "$(dirname "$0")/../.." || exit

# Activate known-good environment
source ~/regolith-robotics-env/bin/activate

# Run controller
python3 Robotics/motion/task_controller.py
