#include "headers/helper_funcs.h"

// Returns the indices of the smallest value in a 2x2 array
short int* minInd(float arr[2][2]) {
    short int i, j;
    float smallest = arr[0][0];
    static short int indices[2];
    for (i=0; i<2; i++) {
        for (j=0; j<2; j++) {
            if (smallest > arr[i][j]) {
                smallest = arr[i][j];
                indices[0] = i;
                indices[1] = j;
            }
        }
    }
    return indices;
}

int sign(float num) {
    return (num >= 0) - (num < 0);
}

bool isRGVAClosest(std::vector<double> droneState, std::vector<double> rgvAPosition, std::vector<double> rgvBPosition) {
    /* Returns true if the closest RGV to the drone is RGV-A
     * Input:  
     *         droneState - 1x4 double vector of current drone position
     *         rgvAPosition   - 1x4 vector of current RGV-A position
     *         rgvBPosition   - 1x4 vector of current RGV-B position
     * Output: 
     *         isRGVAClosest - bool true or false
     */

    double distA, distB;
    // get the x-y plane 2d distance from the drone to each RGV
    distA = pow(pow(droneState[0]-rgvAPosition[0], 2) + pow(droneState[1]-rgvAPosition[1], 2), 0.5);
    distB = pow(pow(droneState[0]-rgvBPosition[0], 2) + pow(droneState[1]-rgvBPosition[1], 2), 0.5);

    // return true if the distance from the drone to RGV-A is smaller than the distance to RGV-B
    return (distA <= distB);
}
