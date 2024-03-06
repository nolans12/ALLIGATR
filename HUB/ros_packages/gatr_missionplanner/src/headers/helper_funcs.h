#pragma once
#include "uas.h"
#include "environment.h"
#include <math.h>

// Helper functions

// @brief Returns the index of the minimum value in a 2x2 array
short int* minInd(float arr[2][2]);

// @brief Returns the sign of a number
int sign(float num);

// @brief Returns true if RGV-A is the closest RGV to the drone
//bool isRGVAClosest(uas drone, environment env);