#include "headers/environment.h"

environment::environment() {
    // time = 0.;
    // timeStep = 1.;
    // Bounds are in the form {{xMin, yMin, zMin}, {xMax, yMax, zMax}}
    bounds = {{0.0, 0.0, 30.0 * 0.3048}, {150.0 * 0.3048, 150.0 * 0.3048, 60.0 * 0.3048}};
    rgvAPosition = {14.0, 9.0, 0.0, 0.0};
    rgvBPosition = {27.0, 35.0, 0.0, 0.0};
    homePosition = {-30.0, 0.0};
    rgvAInView = false;
    rgvBInView = false;
    rgvACoarseComplete = false;
    rgvBCoarseComplete = false;
    rgvAFineComplete = false;
    rgvBFineComplete = false;
    jointComplete = false;

    slt.x_cell = 0;
    slt.y_cell = 0;
    
    slt.num_cells_x = 2;
    slt.num_cells_y = 5;

    //This line starts the search off at the 1,1 cell
    slt.x_iter = slt.num_cells_x-1;
    slt.y_iter = slt.num_cells_y-1;

    slt.recently_transitioned = true;
}

environment::environment(std::vector<std::vector<double>> boundsIn) {
    // time = 0.;
    // timeStep = timeStepIn;
    for (int i = 0; i < 3; i++) {
        bounds[i] = boundsIn[i];
    }
    //rgvAPosition = {{0.0, 0.0, 0.0, 0.0}};
    //rgvBPosition = {{0.0, 0.0, 0.0, 0.0}};
    rgvAInView = false;
    rgvBInView = false;
}

std::vector<double> environment::get_searchpoint() {

    // Define cells as the zone to search. Thus, this will return the center of the cell
    std::vector<double> searchPoint = {0.0, 0.0, 0.0, 0.0};
    searchPoint[0] = bounds[0][0] + (bounds[1][0] - bounds[0][0]) * (double(slt.x_cell) + 0.5) / double(slt.num_cells_x);
    searchPoint[1] = bounds[0][1] + (bounds[1][1] - bounds[0][1]) * (double(slt.y_cell) + 0.5) / double(slt.num_cells_y);
    searchPoint[2] = bounds[0][2]; // This is the lower bound of the z axis
    searchPoint[3] = slt.yaw; // This is the yaw at which the search point is defined

    // If only two points in the x axis, then the search point is the bounds of the x axis
    if (slt.num_cells_x == 2)
    {
        double edge = 4;
        if (slt.x_iter % 2 != 0)
        {
            searchPoint[0] = bounds[0][0] + edge;
        }
        else
        {
            searchPoint[0] = bounds[1][0] - edge;
        }
    }

    return searchPoint;
}

std::vector<double> environment::next_searchpoint() {
    std::vector<double> searchPoint = {0.0, 0.0, 0.0, 0.0};
    // Increment the search location tracker

    // The drone is at the edge of the x axis
    if (slt.x_iter%(slt.num_cells_x-1) == 0 && !slt.recently_transitioned)
    {
        // Move the y_cell component of the search location tracker
        slt.y_iter += 1;
        int y_cell_prev = slt.y_cell;
        slt.y_cell = abs((slt.y_iter % (2*(slt.num_cells_y-1)))-(slt.num_cells_y-1));
        slt.yaw = sign(slt.y_cell-y_cell_prev)*90 - 90;
        slt.recently_transitioned = true;
    } 

    // The drone is not at the edge of the x axis
    else
    {
        // Move the x_cell component of the search location tracker
        slt.x_iter += 1;
        int x_cell_prev = slt.x_cell;
        slt.x_cell = abs((slt.x_iter % (2*(slt.num_cells_x-1)))-(slt.num_cells_x-1));
        slt.yaw = -sign(slt.x_cell-x_cell_prev)*90;
        slt.recently_transitioned = false;
    }

    // Calculate the search point
    searchPoint = get_searchpoint();
    return searchPoint;
}

// Test the next_searchpoint function
void environment::test_next_searchpoint() {
    std::vector<double> searchPoint = {0., 0., 0.};
    for (int i = 0; i < 25; i++) {
        searchPoint = next_searchpoint();
        std::cout << "Search Point: " << searchPoint[0] << ", " << searchPoint[1] << ", " << searchPoint[2] << std::endl;
    }
}