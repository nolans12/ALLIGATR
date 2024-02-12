# Build ROS workspace from scratch
# Run this if problems are encountered with ROS not finding a node or something
# This will put a fresh Melodic ws install on the root directory and wipe whatever may have been there previously

#!/bin/bash

CURRENT_DIR=$(pwd) #Save the current directory as a variable

cd
rm catkin_ws -rf
mkdir -p catkin_ws/src
cd catkin_ws

# This is only needed for the first time use of catkin_make
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

cd src
ln -s ${CURRENT_DIR}/ros_packages
cd ..

catkin_make
source devel/setup.bash
cd ${CURRENT_DIR} #Go back to the build directory

