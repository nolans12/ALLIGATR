#pragma once
#include <string>

struct phaseStep {
    // time at this step [s]
    float time;
    // phase (e.g. "Takeoff") at this step
    std::string phase;
    phaseStep* next;
};