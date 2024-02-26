#include "headers/environment.h"

environment::environment() {
    int i;
    time = 0.;
    timeStep = 1.;
    bounds[0] = 0.;
    bounds[1] = 150.;
    bounds[2] = 0.;
    bounds[3] = 150.;
    bounds[4] = 30.;
    bounds[5] = 60.;
    for (i = 0; i < 3; i++) {
        rgvAPosition[i] = 0.;
        rgvBPosition[i] = 0.;
    }
    rgvAInView = false;
    rgvBInView = false;
}

environment::environment(float boundsIn[6], float timeStepIn) {
    int i;
    time = 0.;
    timeStep = timeStepIn;
    for (i = 0; i < 6; i++) {
        bounds[i] = boundsIn[i];
    }
    for (i = 0; i < 3; i++) {
        rgvAPosition[i] = 0.;
        rgvBPosition[i] = 0.;
    }
    rgvAInView = false;
    rgvBInView = false;
}