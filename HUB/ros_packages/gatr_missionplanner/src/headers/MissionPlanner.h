#pragma once
#include "gnc_functions.hpp"
#include "uas.h"
#include "environment.h"
#include <vector>
#include <iomanip>

class MissionPlanner {
    public:
        MissionPlanner();

        /////////// Testing Modes /////////////////////////
        // Makes the drone fly in a square pattern around the environment bounds
        std::vector<float> bounds_trace(std::vector<float> waypoint);

        /////////// Search Phase //////////////////////////
        std::vector<float> search(std::vector<float> waypoint);

        /////////// Course Localization Phase /////////////

        /////////// Fine Localization Phase ///////////////

        /////////// Joint Localization Phase //////////////

        /////////// Helper Functions //////////////////////
        
        // Output the current state of the drone
        void output_drone_state();

        // Update the state of the drone
        void update_drone_state();

    
    private:
        uas drone;
        environment env;
};