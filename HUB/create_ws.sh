# Build ROS workspace from scratch
# Run this if problems are encountered with ROS not finding a node or something
# This will put a fresh Melodic ws install on the root directory and wipe whatever may have been there previously

#!/bin/bash

CURRENT_DIR=$(pwd) #Save the current directory as a variable
ARG1=$1 #Build the catkin workspace without the IQ ROS package WARNING: this will delete the IQ package from the catkin workspace

cd
rm ~/catkin_ws -rf
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

# This is only needed for the first time use of catkin_make
#catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
catkin init

#################### MAVROS STEPS #####################
# Make sure all the dependencies are installed from this website first
# https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_ros.md

# If the first argument is 'bare' then it will skip the MAVROS installation
if [ "$ARG1" != "bare" ]; then
    cd ~/catkin_ws
    wstool init ~/catkin_ws/src

    rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
    rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
    wstool merge -t src /tmp/mavros.rosinstall
    wstool update -t src
    rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

    catkin build

    source ~/catkin_ws/devel/setup.bash
    source ~/.bashrc

    # The GeoGraphicLib is important, but this line doesn't seem to work
    #sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh

    # Try following these steps if you need to download GeographicLib
    # https://docs.px4.io/main/en/ros/mavros_installation.html

    cd ~/catkin_ws/src
    git clone https://github.com/Intelligent-Quads/iq_sim.git  #Need for Gazebo sims
    #git clone https://github.com/Intelligent-Quads/iq_gnc.git  #Need for Gazebo GNC, copy included however in repo under ros_packages

    
fi

#######################################################

cd src
ln -s ${CURRENT_DIR}/ros_packages
cd ..

catkin build #Use catkin_make if not using MAVROS and catkin build if using MAVROS
source devel/setup.bash
source ~/.bashrc
cd ${CURRENT_DIR} #Go back to the build directory

#################### ADDITIONAL STEPS #####################
    # By adding this file to your source, there won't be issues finding ROS packages
    RED='\033[0;31m'
    echo -e "${RED}WARNING: Do "echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc" If this is your first run on this computer"
    echo -e "${RED}WARNING: Do "echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/iq_sim/models" >> ~/.bashrc" If this is your first run on this computer"

