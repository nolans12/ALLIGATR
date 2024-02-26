#pragma once
#include <string>
#include "phaseStep.h"
#include "pathStep.h"

class uas {
    public:
        

    private:
        float* state; // Why is this a pointer?


        float epsilon, fovNarrow, fovWide, theta, thetaJoint, jointTime;
        bool jointComplete;
        std::string status;
        pathStep* pathRoot;
        pathStep* pathNow;
        phaseStep* phaseRoot;
        phaseStep* phaseNow;
        unsigned short int p;

        uas();
};