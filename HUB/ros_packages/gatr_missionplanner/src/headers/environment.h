#pragma once
#include <vector>

class environment {
    public:
        //float time, timeStep;

        // Bounds defined as [ [Xmin, Ymin, Zmin], [Xmax, Ymax, Zmax]]
        std::vector<std::vector<float>> bounds;

        // RGV positions defined as [X, Y, Z]
        std::vector<double> rgvAPosition;
        std::vector<double> rgvBPosition;
        bool rgvAInView, rgvBInView;

        environment();
        environment(std::vector<std::vector<float>> boundsIn);
};