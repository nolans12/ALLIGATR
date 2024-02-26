#pragma once
#include <string>
#include <vector>
//#include "phaseStep.h"
//#include "pathStep.h"

class uas {
public:
    /*  epsilon       -  allowable positional error, used in reachedPoint [ft]
     *  state         -  1x3 array containing current uas position [ft]
     *  pathRoot      -  points to the root of pathStep linked list where each node has a 1x3 array of the commanded point [ft]
     *  pathNow       -  points to the most recent entry of the pathStep linked list
     *  fovNarrow     -  narrow FOV of the camera [deg]
     *  fovWide       -  wide FOV of the camera [deg]
     *  p             -  keeps track of search iteration, used in SearchPath
     *  theta         -  keeps track of orbiting flight path iteration, used in coarsePath [deg]
     *  thetaJoint    -  keeps track of angle for joint phase
     *  jointTime     -  tracks how much time has been spent collecting data in joint phase [s]
     *  jointComplete -  boolean holding whether the joint phase has been completed or not
     *  phaseRoot     -  points to the root of phaseStep linked list where each node has the time [s] and a string of the phase
     *  phaseNow      -  points to the most recent entry of the phaseStep linked list
     *  status        -  holds the current phase of the mission string (may be redundant with phaseNow - track in further implementation)
     */
    std::vector<float> state;
    float epsilon, fovNarrow, fovWide, theta, thetaJoint, jointTime;
    bool jointComplete;
    std::string status;
    // pathStep* pathRoot;
    // pathStep* pathNow;
    // phaseStep* phaseRoot;
    // phaseStep* phaseNow;
    unsigned short int p;

    uas(); //Default Constructor

    /// @brief Gets the dimensions of the field of view of the camera when projected onto the ground
    std::vector<float> getFOVDims();

    /// @brief Checks if the UAS has reached a commanded point
    bool reachedPoint(std::vector<float> path);
};