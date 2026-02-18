#!/bin/bash

# Always run from project root
cd ~/sidequest || exit

# Activate virtual environment
source ~/regolith-robotics-env/bin/activate

# Run the task controller
python3 task_controller.py

