#!/bin/bash

# Initialize our own variables
# Build flag is used to determine if the ROS packages should be built before execution. Use if changes to ros_packages are made
BUILD_FLAG=0

# A POSIX variable
OPTIND=1         # Reset in case getopts has been used previously in the shell.

# Parse command line arguments. This will check if the -b command line input is set
while getopts "b" opt; do
    case "$opt" in
    b)  BUILD_FLAG=1
        ;;
    esac
done

# Housekeeping, this will shift the command line arguments so that $1 refers to the first argument, $2 to the second, and so on.
shift $((OPTIND-1))
[ "${1:-}" = "--" ] && shift

# Sets the font to be bigger on Xterm
xrdb -merge ~/.Xresources

# Save the file name as a variable called SOURCE_FILE
SOURCE_FILE="src/main.cpp"

# Function to kill all child processes when script exits
function cleanup {
    pkill -P $$
}
trap cleanup EXIT

# Create a new catkin_ws if one doesnt exist
# if [ -d "~/catkin_ws" ]; then
#     echo "Catkin Workspace Detected!"
# else
#     bash src/build_catkin_ws.sh
# fi


# This will clear any previous builds before building again
rm build -rf
mkdir -p build

#Compile the ROS packages
CURRENT_DIR=$(pwd) #Save the current directory as a variable
cd ~/catkin_ws

# Build the catkin_ws only if the build flag is set
if [ $BUILD_FLAG -eq 1 ]; then
    catkin build
fi

source devel/setup.bash
cd ${CURRENT_DIR} #Go back to the build directory

run_node()
{
    PKG_NAME=$1
    NODE_NAME=$2
    NODE_TITLE=$3

    # Make sure the node being ran is an executable
    cd ${CURRENT_DIR}/ros_packages/${PKG_NAME}
    chmod +x scripts/${NODE_NAME}
    chmod +x src/${NODE_NAME}
    cd ${CURRENT_DIR}

    # Launch the XTERM terminal and run the ROS node
    # Make copies of this line of code for any additional nodes
    xterm -T $NODE_TITLE -e "source ~/.bashrc; rosrun $PKG_NAME $NODE_NAME; exec bash" &
    sleep 1
}

#Opens a roscore terminal. If one already exists, it will close itself
xterm -e "source ~/.bashrc; roscore; exit; exec bash" &
sleep 2


################### ADD PROCESSES HERE ######################
# Goes in the form of run_node <package_name> <node_name> <node_title>

# AR Detection Node
run_node gatr_computer_vision blob_detection_node.py Blob_Detection_Node

# Blob Detection Node
run_node gatr_computer_vision ARtag_node.py AR_Tag_Detection_Node

# Mision Planner Node
run_node gatr_missionplanner mp_node Mission_Planner_Node

#Start the MAVROS node
xterm -T "mavros" -e "roslaunch iq_sim apm.launch" &

#############################################################

#Make the project using the passed in source file
cd build
cmake -DSOURCE_FILE=${SOURCE_FILE} ..
make
./main
cd ..
