#include "headers/reachedPoint.h"
#include <stdlib.h>

bool reachedPoint(uas drone, float* path) {
    return (abs(drone.state[0]-path[0]) < drone.epsilon) && (abs(drone.state[1]-path[1]) < drone.epsilon) && (abs(drone.state[2]-path[2]) < drone.epsilon);
}