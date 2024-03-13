#pragma once
#include "gnc_functions.hpp"
#include "uas.h"
#include "environment.h"
#include <vector>
#include <iomanip>
#include <iostream>
#include <Eigen>

class MissionPlanner {
    public:
        MissionPlanner();

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
        std::string boundary_control_phase();

        /////////// Search Phase //////////////////////////
        std::vector<double> search_motion(std::vector<double> waypoint);
        std::string search_phase();

        /////////// Trail Phase ///////////////////////////
        std::vector<double> trail_motion(std::vector<double> waypoint);
        std::string trail_phase();

        /////////// Course Localization Phase /////////////
        std::vector<double> coarse_motion(std::vector<double> waypoint);
        std::string coarse_phase();

        /////////// Fine Localization Phase ///////////////
        std::vector<double> fine_motion(std::vector<double> waypoint);
        std::string fine_phase();

        /////////// Joint Localization Phase //////////////
        std::vector<double> joint_motion(std::vector<double> waypoint);
        std::string joint_phase();

        /////////// Helper Functions //////////////////////
        double getYaw(std::vector<double> waypoint);

        std::vector<double> cross(std::vector<double> const &a, std::vector<double> const &b);

        double dot(std::vector<double> const &a, std::vector<double> const &b);

        double norm(std::vector<double> const &a);
        
        // Output the current state of the drone
        void output_drone_state();

        // Update the state of the drone
        void update_drone_state(std::vector<double> waypoint);

        // Check if a waypoint would fall outside of the environment bounds
        bool out_of_bounds(std::vector<double> waypoint);

        // Check if an RGV is detected through the AR tag node
        bool RGV_detected();

        bool isRGVAClosest();

        std::string getPhase();
        
    private:
        std::string phase;
        uas drone;
        environment env;

};