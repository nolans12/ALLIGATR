#include "headers/getFOVDims.h"
#include <math.h>

float* getFOVDims(uas drone) {
    float L, W;
    static float dims[2];
    L = tan(M_PI/360.*drone.fovWide)*drone.state[2];
    W = tan(M_PI/360*drone.fovNarrow)*drone.state[2];
    dims[0] = L;
    dims[1] = W;
    return dims;
}