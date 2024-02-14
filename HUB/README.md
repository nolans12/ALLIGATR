# All Commands

`bash create_ws.sh`: Configures the ROS workspace for system (ASSUMES DEPENDANCIES INSTALLED)

`bash create_ws.sh bare`: Configures a bare-bones ROS workspace (DOES NOT INCLUDE SIM AND MAVROS)

`bash engage.sh main.cpp`: Executes the autopilot + system checks (FOR JETSON NANO).

`bash engage.sh -b main.cpp`: Builds and executes the autopilot + system checks. Use when changes are made in `ros_packages` (FOR JETSON NANO).

`bash gazebo_engage.sh main.cpp`: Builds and executes the autopilot in a gazebo simulation. (FOR SITL)

# Operating the HUB

**Building the HUB**: The HUB needs to have a 'catkin_ws' compiled in the home directory for ROS to work, a main component of the HUB. To automatically build this, run `bash create_ws.sh` while in the HUB directory. If an error is encountered, you can create it without MAVROS by running `bash create_ws.sh bare`. This will likely lead to errors later on however unless the workspace has been configured followign these steps: https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_ros.md. 

After building the workspace, if it is the first time on a fresh install, you need to also manually run these commands:
`echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc` then `echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/iq_sim/models" >> ~/.bashrc`. You can check if you already did this by running `more ~/.bashrc` and scrolling to the bottom to see if the lines have been copied to your .bashrc file.

Note: A lot of github repos will be cloned in the build process, it may be beneficial to set up an ssh key linked to your github account if you run into errors with github during this process.

**Engaging the Drone**: From the HUB directory run `bash engage.sh -b main.cpp` to begin the autonomous procedure laid out in the main.cpp script. To run a simulation of the mission, run `bash gazebo_engage.sh main.cpp`.

# Authors:

**TEAM ALLIGATR - University of Colorado Boulder AES Senior Projects 2024**

**Carson Kohlbrenner**: Interfacing and compiling

**Thomas Dunnington**: Computer vision

**Zane Vandivere**: Mission planning

**Nolan Stevenson**: Simulation and mission planning