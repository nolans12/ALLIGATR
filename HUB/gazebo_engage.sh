#!/bin/bash

# Get the command line arguments if there are any

# Sets the font to be bigger on Xterm
xrdb -merge ~/.Xresources

# Save the file name as a variable called SOURCE_FILE
SOURCE_FILE="src/main.cpp"

# Function to kill all child processes when script exits
function cleanup {
    # Send a shutdown request to all running ROS nodes
    rosnode kill -a

    # Wait for a bit to allow the ROS nodes to shutdown
    sleep 2

    # Kill all child processes
    pkill -P $$ -2

    # Wait for all child processes to exit
    wait
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

# Use catkin_make if not using MAVROS and catkin build if using MAVROS
# catkin_make
catkin build

# If the build failed, exit the script
if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

source devel/setup.bash
cd ${CURRENT_DIR} #Go back to the build directory

run_node()
{
    PKG_NAME=$1
    NODE_NAME=$2

    # Make sure the node being ran is an executable
    cd ${CURRENT_DIR}/ros_packages/${PKG_NAME}
    chmod +x scripts/${NODE_NAME}
    chmod +x src/${NODE_NAME}
    cd ${CURRENT_DIR}

    # Launch the XTERM terminal and run the ROS node
    # Make copies of this line of code for any additional nodes
    xterm -geometry 80x10 -e "source ~/.bashrc; rosrun $PKG_NAME $NODE_NAME; exec bash" &
    sleep 1
}

#Opens a roscore terminal. If one already exists, it will close itself
xterm -e "source ~/.bashrc; roscore; exit; exec bash" &
sleep 2

#Get the screen size so windows can display properly
Xaxis=$(xrandr --current | grep '*' | uniq | awk '{print $1}' | cut -d 'x' -f1)

Yaxis=$(xrandr --current | grep '*' | uniq | awk '{print $1}' | cut -d 'x' -f2)


################### ADD PROCESSES HERE ######################

#run_node gatr_computer_vision blob_detection_node.py
#run_node gatr_computer_vision ARtag_node.py

#Start the Gazebo simulation
xterm -geometry 40x10 -T "Simulation Host" -e "WID=\$(xdotool getactivewindow); xdotool windowminimize \$WID; roslaunch gatr_missionplanner multi_rgv_stationary.launch" & 
sleep 5

#Start the ArduPilot Plugin for Software in the Loop (SITL)
xterm -geometry 80x10 -T "COMMAND PLUGIN" -e "cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console" &
sleep 3

#Start the MAVROS node
xterm -geometry 40x10 -T "mavros" -e "roslaunch iq_sim apm.launch" &
#xterm -e "roslaunch iq_sim apm.launch" &
sleep 3

#Start the Camera Viewer node
#xterm -geometry 40x10 -T "Camera Viewer" -e "rosrun rqt_image_view rqt_image_view image:=/webcam/image_raw/compressed" &
#xterm -e "roslaunch iq_sim apm.launch" &
#sleep 3

# Primary sensor node, running AR tag detection
run_node gatr_computer_vision primary_node.py Primary_Detection_Node

run_node gatr_computer_vision ARtag_node.py AR_Tag_Detection_Node

run_node gatr_computer_vision localize_node.py Localization_Node

run_node gatr_missionplanner move_jackals_node.py Move_Jackals_Node

run_node gatr_missionplanner data_logger_node.py Data_Logger_Node

#Start the Mission Planner ($1 is a command line argument for the mission pattern to load)
if [ -z "$1" ]; then
    xterm -hold -geometry 120x10 -T "MISSION PLANNER" -e "rosrun gatr_missionplanner mp_node" &
else
    xterm -hold -geometry 120x10 -T "MISSION PLANNER" -e "rosrun gatr_missionplanner mp_node $1" &
fi
sleep 3

# Speak the output of the mission planner
run_node gatr_speak hes_alive_2.py GATR_GPT

#############################################################

#Make the project using the passed in source file
cd build
cmake -DSOURCE_FILE=${SOURCE_FILE} ..
make
./main
cd ..
