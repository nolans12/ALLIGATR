ALLIGATR

Repo for code that will run on the Jetson Nano

# HUB Dependancies
-Cmake 3.10 (or later), https://medium.com/@rahulbagul330/how-to-install-cmake-on-ubuntu-18-04-linux-c585394226bf

-C++ compiler (Tested with GCC), https://code.visualstudio.com/docs/cpp/config-linux

-ROS Melodic, https://wiki.ros.org/melodic/Installation/Ubuntu
-- Follow all the way through building the catkin workspace

-Ubuntu 18.04, 

-Xterm, Install with `sudo apt update` then `sudo apt-get install xterm`. This is used to open up new kernals that will run the ROS packages seperately.

-ArduPilot and MavProxy, https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot.md

# Gazebo Dependancies

To check if Gazebo is working:
1. In terminal one, run
   ```
   gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
   ```
2. In terminal two, run
   ```
   cd ~/ardupilot/ArduCopter/
   sim_vehicle.py -v ArduCopter -f gazebo-iris --console
   ```

   In the case of a symbol lookup error, try running this command `sudo apt upgrade libignition-math2`

-Gazebo and Ardupilot plugin, https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_gazebo_arduplugin.md
    --Incase of error installing GeographicLib (OS not supported), follow this guide too https://docs.px4.io/main/en/ros/mavros_installation.html

# Computer Vision Dependancies

