#pragma once
#include <iostream>
#include "uas.h"
#include "environment.h"
#include <gnc_functions.hpp>
#include <vector>

class MissionPlanner {
    public:
        MissionPlanner();

        // Search phase
        std::vector<float> search(std::vector<float> path);

        std::vector<float> bounds_trace(std::vector<float> path);

        // Course Localization Phase

        // Fine Localization Phase

        // Joint Localization Phase
    
    private:
        uas drone;
        environment env;
};