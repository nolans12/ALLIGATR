#pragma once
#include <vector>
#include <string>
#include <iostream>
#include <cmath>
//#include "phaseStep.h"
//#include "pathStep.h"

class uas {
public:
    /*  state         -  1x4 double vector containing current uas position [ft] and yaw
     *  epsilon       -  allowable positional error, used in reachedPoint [ft]
     *  fovNarrow     -  narrow FOV of the camera [deg]
     *  fovWide       -  wide FOV of the camera [deg]
     *  theta         -  keeps track of orbiting flight path iteration, used in coarsePath [deg]
     *  thetaJoint    -  keeps track of angle for joint phase
     *  courseATime   -  tracks how much time has been spent collecting data on RGV-A in coarse phase [s]
     *  courseBTime   -  tracks how much time has been spent collecting data on RGV-B in coarse phase [s]
     *  fineATime     -  tracks how much time has been spent collecting data on RGV-A in fine phase [s]
     *  fineBTime     -  tracks how much time has been spent collecting data on RGV-B in fine phase [s]
     *  jointTime     -  tracks how much time has been spent collecting data in joint phase [s]
     *  status        -  holds the current phase of the mission string (may be redundant with phase - track in further implementation)
     */
    std::vector<double> state;
    std::vector<double> dest;
    std::vector<double> home;
    //std::vector<double> vel;
    double epsilon, fovNarrow, fovWide, theta, coarseATime, coarseBTime, fineATime, fineBTime, jointTime, orbit_radius, theta_step;
    double trail_altitude, coarse_altitude, fine_altitude;
    double coarse_duration, fine_duration, joint_duration, detection_duration, boundary_duration, search_point_duration, joint_detection_duration;
    double coarse_reset_time, fine_reset_time, joint_reset_time;
    bool jointComplete;
    std::string status;

    int time_history_average; // Number of time steps to average for RGV position
    double stopped_threshold; // Standard deviation of the RGV position estimate to accept stationary RGV condition

    unsigned short int p; //Used only in the bounds trace mode

    uas(); //Default Constructor

    /// @brief Gets the dimensions of the field of view of the camera when projected onto the ground
    std::vector<float> getFOVDims();

    /// @brief Checks if the UAS has reached a commanded point
    bool reachedPoint(std::vector<float> path);
};