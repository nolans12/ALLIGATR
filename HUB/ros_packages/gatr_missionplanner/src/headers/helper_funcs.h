#pragma once
#include "uas.h"
#include "environment.h"
#include <math.h>
#include <cmath>
#include <vector>
//#include <eigen3/Eigen/Dense>

// Helper functions

// @brief Returns the index of the minimum value in a 2x2 array
short int* minInd(float arr[2][2]);

// @brief Returns the sign of a number
int sign(float num);

// @brief Returns true if RGV-A is the closest RGV to the drone
//bool isRGVAClosest(std::vector<double> droneState, std::vector<double> rgvAPosition, std::vector<double> rgvBPosition);

std::vector<double> cross(std::vector<double> const &a, std::vector<double> const &b);

double dot(std::vector<double> const &a, std::vector<double> const &b);

double norm(std::vector<double> const &a);
