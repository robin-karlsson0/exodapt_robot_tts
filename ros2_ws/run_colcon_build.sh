#!/bin/bash

source /home/$USER/.pyenv/versions/robot_tts/bin/activate

export PYTHONPATH=$PYTHONPATH:/home/$USER/.pyenv/versions/robot_tts/lib/python3.10/site-packages
export PYTHONPATH=$PYTHONPATH:/home/$USER/projects/robot_tts/python-clients

colcon build --symlink-install
