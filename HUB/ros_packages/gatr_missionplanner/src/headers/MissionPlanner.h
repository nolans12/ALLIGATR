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

        /////////// Return to Ground Station //////////////
        std::vector<double> go_home_motion(std::vector<double> waypoint);

        /////////// Computer Vision + ROS ///////////////////
        void rgvA_detected_callback(const std_msgs::Float32MultiArray::ConstPtr& coords);
        void rgvB_detected_callback(const std_msgs::Float32MultiArray::ConstPtr& coords);

        /////////// Helper Functions //////////////////////
        double getYaw(std::vector<double> waypoint);
        
        // Output the current state of the drone
        void output_drone_state();

        // Update the state of the drone
        void update_drone_state(std::vector<double> waypoint);

        // Check if a waypoint would fall outside of the environment bounds
        bool out_of_bounds(std::vector<double> waypoint);

        // Check if an RGV is detected through the AR tag node
        bool RGV_detected();

        bool isRGVAClosest();

        std::vector<bool> rgvStopped();

        std::string getPhase();
        void setPhase(std::string phaseIn);
        
    private:
        std::string phase;
        std::vector<std::string> phases;
        unsigned short int smootherCount;
        uas drone;
        environment env;
        ros::Subscriber rel_coord_A_sub;
        ros::Subscriber rel_coord_B_sub;

        ros::Time coarse_time_engaged; // Time the drone started the coarse localization phase
        ros::Time fine_time_engaged; // Time the drone started the fine localization phase
        ros::Time joint_time_engaged; // Time the drone started the joint localization phase
        ros::Time out_of_bounds_time; // Time the drone went out of bounds
        ros::Time search_point_time; // Time the drone started a given target in the search phase. Used to remove an error where the drone would get stuck at a search point

        ros::Duration time_coarsely_localized; // Time the drone has spent in the coarse localization phase if it needs to resume it
        ros::Duration time_finely_localized; // Time the drone has spent in the fine localization phase if it needs to resume it
        ros::Duration time_joint_localized; // Time the drone has spent in the joint localization phase if it needs to resume it

        ros::Time last_rgvA_detection; // Time the last RGV-A detection was made
        ros::Time last_rgvB_detection; // Time the last RGV-B detection was made

        // CSV file to store RGV positions
        std::ofstream rgvA_csv;
        std::ofstream rgvB_csv;
};

