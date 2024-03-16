#pragma once
#include "gnc_functions.hpp"
#include "uas.h"
#include "environment.h"
#include <vector>
#include <iomanip>
#include <math.h>

class MissionPlanner {
    public:
        MissionPlanner();

        /////////// Testing Modes /////////////////////////
        // Makes the drone fly in a square pattern around the environment bounds
        std::vector<double> bounds_trace(std::vector<double> waypoint);

        /////////// Search Phase //////////////////////////
        std::vector<double> search(std::vector<double> waypoint);

        /////////// Coarse Localization Phase /////////////
        std::vector<double> coarse(std::vector<double> waypoint);

        /////////// Fine Localization Phase ///////////////

        /////////// Joint Localization Phase //////////////

        /////////// Helper Functions //////////////////////
        
        // Output the current state of the drone
        void output_drone_state();

        // Update the state of the drone
        void update_drone_state(std::vector<double> waypoint);

        // Check if a waypoint would fall outside of the environment bounds
        bool out_of_bounds(std::vector<double> waypoint);

        uas drone;
        environment env;
        
    private:

};