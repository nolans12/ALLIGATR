#include "headers/uas.h"
//#include "headers/pathStep.h"

uas::uas() {
    /*  epsilon       -  allowable positional error, used in reachedPoint [ft]
     *  state         -  1x4 array containing current uas position and heading[ft]
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
    epsilon = 3;
    state = {0., 0., 0., 0.};
    dest = {0., 0., 0., 0.};
    // pathRoot = new pathStep;
    // pathNow = pathRoot;
    // pathRoot->path[0] = 0.;
    // pathRoot->path[1] = 0.;
    // pathRoot->path[2] = 0.;
    fovNarrow = 67;
    fovWide = 102;

    p = 0;
    theta = -1.0f;
    thetaJoint = 0.0f;
    jointTime = 0.0f;
    jointComplete = false;
    // phaseRoot = new phaseStep;
    // phaseNow = phaseRoot;
    // phaseRoot->time = 0.;
    // phaseRoot->phase = "Takeoff";
    status = "STANDBY";
}

/// @brief Gets the dimensions of the field of view of the camera when projected onto the ground
/// @return dims = [L, W] where L is the length and W is the width of the camera's field of view
std::vector<float> uas::getFOVDims(){
    std::vector<float> dims(2);
    dims[0] = tan(M_PI/360.*fovWide)*state[2];
    dims[1] = tan(M_PI/360*fovNarrow)*state[2];
    return dims;
}

/// @brief Checks if the UAS has reached a commanded point
// bool uas::reachedPoint(std::vector<float> waypoint) {
//     return (abs(state[0]-waypoint[0]) < epsilon) && (abs(state[1]-waypoint[1]) < epsilon) && (abs(state[2]-waypoint[2]) < epsilon);
// }