#include "headers/helper_funcs.h"
#include <math.h>

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

bool isRGVAClosest() {
    double distA, distB;
    distA = pow(pow(drone.state[0]-env.rgvAPostion[0], 2) + pow(drone.state[1]-env.rgvAPosition[1], 2), 0.5);
    distB = pow(pow(drone.state[0]-env.rgvBPostion[0], 2) + pow(drone.state[1]-env.rgvBPosition[1], 2), 0.5);

    return (distA <= distB);
}
