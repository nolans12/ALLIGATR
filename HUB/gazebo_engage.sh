#!/bin/bash

# Save the file name as a variable called SOURCE_FILE
SOURCE_FILE=$1

# Function to kill all child processes when script exits
function cleanup {
    pkill -P $$
}
trap cleanup EXIT

# Create a new catkin_ws if one doesnt exist
if [ -d "~/catkin_ws" ]; then
    echo "Catkin Workspace Detected!"
else
    bash src/build_catkin_ws.sh
fi


# This will clear any previous builds before building again
rm build -rf
mkdir -p build

#Compile the ROS packages
CURRENT_DIR=$(pwd) #Save the current directory as a variable
cd ~/catkin_ws
catkin_make
source devel/setup.bash
cd ${CURRENT_DIR} #Go back to the build directory

run_node()
{
    PKG_NAME=$1
    NODE_NAME=$2

    # Make sure the node being ran is an executable
    cd ${CURRENT_DIR}/ros_packages/${PKG_NAME}
    chmod +x scripts/${NODE_NAME}
    cd ${CURRENT_DIR}

    # Launch the XTERM terminal and run the ROS node
    # Make copies of this line of code for any additional nodes
    xterm -e "source ~/.bashrc; rosrun $PKG_NAME $NODE_NAME; exec bash" &
}

#Opens a roscore terminal. If one already exists, it will close itself
xterm -e "source ~/.bashrc; roscore; exit; exec bash" &


################### ADD PROCESSES HERE ######################

run_node ros_beginner_package first_cpp_node.cpp

#############################################################

#Make the project using the passed in source file
cd build
cmake -DSOURCE_FILE=${SOURCE_FILE} ..
make
./main
cd ..
