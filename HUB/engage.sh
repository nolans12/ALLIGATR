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
# xterm -e "source ~/.bashrc; roscore; exit; exec bash" &
# sleep 2

# Check if an argument was provided
if [ -z "$1" ]; then
    echo "Usage: $0 <ip_address> to connect to roscore ip"
    xterm -e "source ~/.bashrc; roscore; exit; exec bash" &
    
else

    # Define the remote ROS master URI
    REMOTE_ROS_MASTER_URI="http://$1:11311"

    # Export the remote ROS master URI
    export ROS_MASTER_URI=$REMOTE_ROS_MASTER_URI

    # Attempt to connect to the remote ROS master
    # Here we use rostopic list as a method to check connectivity. Adjust the timeout as needed.
    if rostopic list -v --timeout=3; then
        echo "Successfully connected to the remote ROS master at $ROS_MASTER_URI."
    else
        echo "Failed to connect to the remote ROS master. Starting a local roscore."

        # Unset the ROS_MASTER_URI to avoid conflicts
        unset ROS_MASTER_URI

        # Start a local roscore
        # It's recommended to run roscore in the background or in a separate terminal/session
        # because it does not exit until manually stopped.
        xterm -e "source ~/.bashrc; roscore; exit; exec bash" &

        # Capture the PID of the last background process (roscore)
        ROSCORE_PID=$!

        echo "Local roscore started with PID $ROSCORE_PID."

        # Optional: wait or perform additional tasks here

        # If you need to stop the local roscore at the end of this script, use:
        # kill $ROSCORE_PID
    fi
fi


################### ADD PROCESSES HERE ######################
# Goes in the form of run_node <package_name> <node_name> <node_title>

# Blob Detection Node
run_node gatr_computer_vision blob_detection_node.py Blob_Detection_Node

# AR Detection Node
run_node gatr_computer_vision ARtag_node.py AR_Tag_Detection_Node

# Localization Node
run_node gatr_computer_vision localize_node.py Localize_Node

# Mision Planner Node
#run_node gatr_missionplanner mp_node Mission_Planner_Node

# Start the MAVLINK connection to cube, opening on ttyTHS1 port
#xterm -T "mavlink" -e "sudo mavproxy.py --master=/dev/ttyTHS1" &  

#Start the MAVROS node
#xterm -T "mavros" -e "roslaunch mavros apm.launch fcu_url:=/dev/ttyTHS1:57600@" &

#############################################################

#Make the project using the passed in source file
cd build
cmake -DSOURCE_FILE=${SOURCE_FILE} ..
make
./main
cd ..
