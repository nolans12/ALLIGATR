ALLIGATR

Repo for code that will run on the Jetson Nano

# HUB Dependancies
-Cmake 3.10 (or later), https://medium.com/@rahulbagul330/how-to-install-cmake-on-ubuntu-18-04-linux-c585394226bf

-C++ compiler (Tested with GCC), https://code.visualstudio.com/docs/cpp/config-linux

-ROS Melodic, https://wiki.ros.org/melodic/Installation/Ubuntu
Notes: make sure to use `catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3` on first make of catkin_ws since this repo. If you are using python for any of the packages, I have run into errors using python3 regardless of this setup. It is advisable to use python 2 whenever possible. If errors are encountered with running ros packages, make it is linked to the catkin_ws/src folder using `cd ~/catkin_ws/src` then `ln -s ~/Team_ALLIGATR/HUB/ros_packages`. 

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

-Gazebo and Ardupilot plugin, https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_gazebo_arduplugin.md

# Computer Vision Dependancies

