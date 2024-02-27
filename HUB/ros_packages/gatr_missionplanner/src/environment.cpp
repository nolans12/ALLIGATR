#include "headers/environment.h"

environment::environment() {
    // time = 0.;
    // timeStep = 1.;
    bounds = {{0.0f, 0.0f, 30.0f}, {150.0f, 150.0f, 60.0f}};
    rgvAPosition = {0., 0., 0.};
    rgvBPosition = {0., 0., 0.};
    rgvAInView = false;
    rgvBInView = false;
}

environment::environment(std::vector<std::vector<float>> boundsIn) {
    // time = 0.;
    // timeStep = timeStepIn;
    for (int i = 0; i < 3; i++) {
        bounds[i] = boundsIn[i];
    }
    rgvAPosition = {0., 0., 0.};
    rgvBPosition = {0., 0., 0.};
    rgvAInView = false;
    rgvBInView = false;
}