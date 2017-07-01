#!/bin/sh

cd src
git clone https://github.com/ros-perception/image_common.git
git clone https://github.com/ros-drivers/usb_cam.git --branch develop
git clone https://github.com/ros-perception/image_pipeline.git --branch indigo
cd ..
catkin_make
source devel/setup.bash
roslaunch missionpkg missionpkg.launch