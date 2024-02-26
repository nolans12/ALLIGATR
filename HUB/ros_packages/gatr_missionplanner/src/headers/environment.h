#pragma once
#include <vector>


class environment {
    public:
        float time, timeStep;
        std::vector<std::vector<float>> bounds;
        std::vector<float> rgvAPosition;
        std::vector<float> rgvBPosition;
        bool rgvAInView, rgvBInView;

        environment();
        environment(float boundsIn[6], float timeStepIn);
};