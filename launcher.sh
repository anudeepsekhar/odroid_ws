#!/bin/sh

cd odroid_ws
git submodule update --init --recursive
chmod +x src/missionpkg/src/el_det_fi.py
catkin_make
source devel/setup.bash
roslaunch missionpkg missionpkg.launch
