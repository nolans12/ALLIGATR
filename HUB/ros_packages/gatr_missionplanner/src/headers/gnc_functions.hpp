#ifndef GNC_FUNCTIONS_HPP
#define GNC_FUNCTIONS_HPP

#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>
#include <string>

struct gnc_api_waypoint{
	float x; ///< distance in x with respect to your reference frame
	float y; ///< distance in y with respect to your reference frame
	float z; ///< distance in z with respect to your reference frame
	float psi; ///< rotation about the third axis of your reference frame
};

//get armed state
void state_cb(const mavros_msgs::State::ConstPtr& msg);

geometry_msgs::Point enu_2_local(nav_msgs::Odometry current_pose_enu);
std::vector<double > enu_2_local(std::vector<double> current_pose_enu);
std::vector<double> local_2_enu(std::vector<double> current_pose_local);

//get current position of drone
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg);

geometry_msgs::Point get_current_location();
float get_current_heading();


//set orientation of the drone (drone should always be level) 
// Heading input should match the ENU coordinate system
/**
\ingroup control_functions
This function is used to specify the drone’s heading in the local reference frame. Psi is a counter clockwise rotation following the drone’s reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone. 
@returns n/a
*/
void set_heading(float heading);
// set position to fly to in the local frame
/**
\ingroup control_functions
This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the local reference frame. This is typically defined from the location the drone is launched. Psi is counter clockwise rotation following the drone’s reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone. 
@returns n/a
*/
void set_destination(float x, float y, float z, float psi);
void set_destination_lla(float lat, float lon, float alt, float heading);
void set_destination_lla_raw(float lat, float lon, float alt, float heading);
/**
\ingroup control_functions
Wait for connect is a function that will hold the program until communication with the FCU is established.
@returns 0 - connected to fcu 
@returns -1 - failed to connect to drone
*/
int wait4connect();
/**
\ingroup control_functions
Wait for strat will hold the program until the user signals the FCU to enther mode guided. This is typically done from a switch on the safety pilot’s remote or from the ground control station.
@returns 0 - mission started
@returns -1 - failed to start mission
*/
int wait4start();
/**
\ingroup control_functions
This function will create a local reference frame based on the starting location of the drone. This is typically done right before takeoff. This reference frame is what all of the the set destination commands will be in reference to.
@returns 0 - frame initialized
*/
int initialize_local_frame();

int arm();

/**
\ingroup control_functions
The takeoff function will arm the drone and put the drone in a hover above the initial position. 
@returns 0 - nominal takeoff 
@returns -1 - failed to arm 
@returns -2 - failed to takeoff
*/
int takeoff(float takeoff_alt);
/**
\ingroup control_functions
This function returns an int of 1 or 0. THis function can be used to check when to request the next waypoint in the mission. 
@return 1 - waypoint reached 
@return 0 - waypoint not reached
*/
int check_waypoint_reached(float pos_tolerance=0.3, float heading_tolerance=0.01);
/**
\ingroup control_functions
this function changes the mode of the drone to a user specified mode. This takes the mode as a string. ex. set_mode("GUIDED")
@returns 1 - mode change successful
@returns 0 - mode change not successful
*/
int set_mode(std::string mode);
/**
\ingroup control_functions
this function changes the mode of the drone to land
@returns 1 - mode change successful
@returns 0 - mode change not successful
*/
int land();
/**
\ingroup control_functions
This function is used to change the speed of the vehicle in guided mode. it takes the speed in meters per second as a float as the input
@returns 0 for success
*/
int set_speed(float speed__mps);
int auto_set_current_waypoint(int seq);

/**
\ingroup control_functions
used to set yaw when running lla waypoint missions
param1: Angle				target angle, 0 is north																			deg
param2: Angular Speed		angular speed																						deg/s
param3: Direction			direction: -1: counter clockwise, 1: clockwise					min: -1 max:1 increment:2	
param4: Relative			0: absolute angle, 1: relative offset							min:0 max:1 increment:1	
@returns 0 for success
*/
int set_yaw(float angle, float speed, float dir, float absolute_rel);

int takeoff_global(float lat, float lon, float alt);
/**
\ingroup control_functions
This function is called at the beginning of a program and will start of the communication links to the FCU. The function requires the program's ros nodehandle as an input 
@returns n/a
*/
int init_publisher_subscriber(ros::NodeHandle controlnode);

#endif