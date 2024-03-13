#pragma once
#include <iostream>
#include <vector>
#include "helper_funcs.h"
#include <cmath>

class environment {
    public:
        environment();
        environment(std::vector<std::vector<float>> boundsIn);

        // Bounds defined as [ [Xmin, Ymin, Zmin], [Xmax, Ymax, Zmax]]
        std::vector<std::vector<float>> bounds;

        // RGV positions defined as [ X0, Y0, Z0, t0 ]
        std::vector<double> rgvAPosition;
        std::vector<double> rgvBPosition;
        bool rgvAInView, rgvBInView, rgvACoarseComplete, rgvBCoarseComplete, rgvAFineComplete, rgvBFineComplete;

        // Return the current search destination
        std::vector<double> get_searchpoint();

        // Iterate the search location tracker
        std::vector<double> next_searchpoint();
        void test_next_searchpoint();

        // Automatically determine the most optimal search discretization
        // void auto_discretize_search();

    private:
        // Use this to track the search location for restarting search after a target is found or errors
        struct search_location_tracker{
            int x_cell, y_cell, num_cells_x, num_cells_y, x_iter, y_iter;
            bool recently_transitioned;
        };

        search_location_tracker slt;

        
};