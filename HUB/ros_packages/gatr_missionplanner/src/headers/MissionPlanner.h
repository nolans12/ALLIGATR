#pragma once
#include "gnc_functions.hpp"
#include "uas.h"
#include "environment.h"
#include <vector>
#include <iomanip>
#include <math.h>
#include <iostream>
#include <fstream>

class MissionPlanner {
    public:
        MissionPlanner();
        MissionPlanner(ros::NodeHandle gnc_node);
        ~MissionPlanner();

        // Deteremines the phase
        void determine_phase();
        std::vector<double> determine_motion(std::vector<double> waypoint);

        /////////// Testing Modes /////////////////////////
        // Makes the drone fly in a square pattern around the environment bounds
        std::vector<double> bounds_trace(std::vector<double> waypoint);

        // Directly flies to an RGV
        std::vector<double> direct_locate(std::vector<double> waypoint);

        /////////// Boundary Control //////////////////////
        std::vector<double> boundary_control_motion(std::vector<double> waypoint);
        void boundary_control_phase();

        /////////// Search Phase //////////////////////////
        std::vector<double> search_motion(std::vector<double> waypoint);
        void search_phase();

        /////////// Trail Phase ///////////////////////////
        std::vector<double> trail_motion(std::vector<double> waypoint);
        void trail_phase();

        /////////// Course Localization Phase /////////////
        std::vector<double> coarse_motion(std::vector<double> waypoint);
        void coarse_phase();

        /////////// Fine Localization Phase ///////////////
        std::vector<double> fine_motion(std::vector<double> waypoint);
        void fine_phase();

        /////////// Joint Localization Phase //////////////
        std::vector<double> joint_motion(std::vector<double> waypoint);
        void joint_phase();
        void joint_search_phase();

        /////////// Return Home ////////////////////////
        std::vector<double> return_home_motion(std::vector<double> waypoint);
        void return_home_phase();

        /////////// Computer Vision + ROS ///////////////////
        void rgvA_detected_callback(const std_msgs::Float32MultiArray::ConstPtr& coords);
        void rgvB_detected_callback(const std_msgs::Float32MultiArray::ConstPtr& coords);
        void rgvA_detected_callback_s(const std_msgs::Float32MultiArray::ConstPtr& coords);
        void rgvB_detected_callback_s(const std_msgs::Float32MultiArray::ConstPtr& coords);


        /////////// Helper Functions //////////////////////
        double getYaw(std::vector<double> waypoint);
        
        // Output the current state of the drone
        void output_drone_state();

        // Update the state of the drone
        void update_drone_state(std::vector<double> waypoint);

        // Check if a waypoint would fall outside of the environment bounds
        bool out_of_bounds(std::vector<double> waypoint);

        bool isRGVAClosest();

        // Check if the drone has stopped moving
        bool rgvAStopped();
        bool rgvBStopped();

        std::string getPhase();
        void setPhase(std::string phaseIn);

        void get_current_location_mav(const geometry_msgs::PoseStamped::ConstPtr& msg);

        void set_primary_activated(bool activated);
        void set_secondary_activated(bool activated);

        bool get_primary_activated();
        bool get_secondary_activated();

        void set_cameras(bool primary, bool secondary);

        // Locking functions to avoid localizing both RGVs at the same time
        void set_lock();
        void reset_lock();
        
    private:
        std::string phase;
        std::string thought;
        std::vector<std::string> phases;
        unsigned short int smootherCount;
        uas drone;
        environment env;

        // Bools that determine which cameras are dictating the drone's motion
        bool primary_activated;
        bool secondary_activated;

        // Locking variables that keep the drone from localizing both RGVs at the same time
        std::string target_lock; // The target the drone is currently locked onto

        ros::Subscriber inert_coord_A_sub_primary;
        ros::Subscriber inert_coord_B_sub_primary;
        ros::Subscriber inert_coord_A_sub_secondary;
        ros::Subscriber inert_coord_B_sub_secondary;
        ros::Subscriber uas_state_sub;
        ros::Publisher phase_pub;
        ros::Publisher speak_pub; // Publisher to the speak node

        ros::Time coarse_time_engaged; // Time the drone started the coarse localization phase
        ros::Time fine_time_engaged; // Time the drone started the fine localization phase
        ros::Time joint_time_engaged; // Time the drone started the joint localization phase
        ros::Time out_of_bounds_time; // Time the drone went out of bounds
        ros::Time search_point_time; // Time the drone started a given target in the search phase. Used to remove an error where the drone would get stuck at a search point
        ros::Time joint_last_detection; // Time the last RGV detection was made in the joint phase

        ros::Duration time_coarsely_localized; // Time the drone has spent in the coarse localization phase if it needs to resume it
        ros::Duration time_finely_localized; // Time the drone has spent in the fine localization phase if it needs to resume it
        ros::Duration time_joint_localized; // Time the drone has spent in the joint localization phase if it needs to resume it

        ros::Time last_rgvA_detection; // Time the last RGV-A detection was made
        ros::Time last_rgvB_detection; // Time the last RGV-B detection was made

        // // CSV file to store RGV positions
        // std::ofstream rgvA_csv;
        // std::ofstream rgvB_csv;
        // std::ofstream uas_csv;
        // std::ofstream uas_csv_rgvA;
        // std::ofstream uas_csv_rgvB;
};

