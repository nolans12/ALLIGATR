# Operating the HUB

**Building the HUB**: The HUB needs to have a 'catkin_ws' compiled in the home directory for ROS to work, a main component of the HUB. To automatically build this, run `bash create_ws.sh` while in the HUB directory. If an error is encountered, you can create it without MAVROS by running `bash create_ws.sh bare`. This will likely lead to errors later on however unless the workspace has been configured followign these steps: https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_ros.md. 

**Engaging the Drone**: From the HUB directory run `bash engage.sh main.cpp` to begin the autonomous procedure laid out in the main.cpp script. To run a simulation of the mission, run `bash gazebo_engage.sh main.cpp`.

# Authors:

**TEAM ALLIGATR - University of Colorado Boulder AES Senior Projects 2024**

**Carson Kohlbrenner**: Interfacing and compiling

**Thomas Dunnington**: Computer vision

**Zane Vandivere**: Mission planning

**Nolan Stevenson**: Simulation and mission planning