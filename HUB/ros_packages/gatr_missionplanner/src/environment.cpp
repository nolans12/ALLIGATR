#include "headers/environment.h"

environment::environment() {
    // time = 0.;
    // timeStep = 1.;
    // Bounds are in the form {{xMin, yMin, zMin}, {xMax, yMax, zMax}}
    bounds = {{0.0f, 0.0f, 30.0f * 0.3048f}, {150.0f * 0.3048f, 150.0f * 0.3048f, 60.0f * 0.3048f}};
    rgvAPosition = {0.0, 0.0, 0.0};
    rgvBPosition = {0.0, 0.0, 0.0};
    rgvAInView = false;
    rgvBInView = false;

    slt.x_cell = 0;
    slt.y_cell = 0;
    
    slt.num_cells_x = 3;
    slt.num_cells_y = 3;

    // Need this statement to start search going in the positive direction
    slt.x_iter = slt.num_cells_x;
    slt.y_iter = slt.num_cells_y;

    slt.recently_transitioned = true;
}

environment::environment(std::vector<std::vector<float>> boundsIn) {
    // time = 0.;
    // timeStep = timeStepIn;
    for (int i = 0; i < 3; i++) {
        bounds[i] = boundsIn[i];
    }
    rgvAPosition = {0.0, 0.0, 0.0};
    rgvBPosition = {0.0, 0.0, 0.0};
    rgvAInView = false;
    rgvBInView = false;
}

std::vector<double> environment::get_searchpoint() {

    // Define cells as the zone to search. Thus, this will return the center of the cell
    std::vector<double> searchPoint = {0., 0., 0.};
    searchPoint[0] = bounds[0][0] + (bounds[1][0] - bounds[0][0]) * (slt.x_cell + 0.5) / slt.num_cells_x;
    searchPoint[1] = bounds[0][1] + (bounds[1][1] - bounds[0][1]) * (slt.y_cell + 0.5) / slt.num_cells_y;
    searchPoint[2] = bounds[0][2]; // This is the lower bound of the z axis
    return searchPoint;
}

std::vector<double> environment::next_searchpoint() {
    std::vector<double> searchPoint = {0., 0., 0.};
    // Increment the search location tracker

    // The drone is at the edge of the x axis
    if (slt.x_cell%(slt.num_cells_x-1) == 0)
    {
        // Move the y_cell component of the search location tracker
        slt.y_iter += 1;
        slt.y_cell == abs((slt.y_iter % (2*slt.num_cells_y)-1)-slt.num_cells_y);
        slt.recently_transitioned = true;
    } 

    // The drone is at the edge of the x axis but already transitioned its y_cell
    // else if (slt.x_cell%slt.num_cells_x == 0 && slt.recently_transitioned)
    // {
    //     // Use the sign of a sine wave with a period of num_cells_x to determine the direction of the search
    //     slt.x_iter += 1;
    //     slt.x_cell = abs((slt.x_iter % (2*slt.num_cells_x))-slt.num_cells_x);
    //     slt.recently_transitioned = false;
    // }

    // The drone is not at the edge of the x axis
    // else
    // {
        // Use the sign of a sine wave with a period of num_cells_x to determine the direction of the search
        slt.x_iter += 1;
        slt.x_cell = abs((slt.x_iter % (2*slt.num_cells_x)-2)-(slt.num_cells_x-1));
    // }


    std::cout << "slt.x_cell: " << slt.x_cell << " slt.y_cell: " << slt.y_cell << std::endl;
    std::cout << "slt.x_iter: " << slt.x_iter << " slt.y_iter: " << slt.y_iter << std::endl;

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