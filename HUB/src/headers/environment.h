#pragma once

class environment {
public:
    float time, timeStep;
    float bounds[6];
    float rgvAPosition[3];
    float rgvBPosition[3];
    bool rgvAInView, rgvBInView;

    environment();
    environment(float boundsIn[6], float timeStepIn);
};