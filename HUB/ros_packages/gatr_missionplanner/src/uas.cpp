#include "headers/uas.h"
//#include "headers/pathStep.h"

uas::uas() {
    /*  state         -  1x4 double vector containing current uas position [ft] and yaw
     *  phase         -  1x5 string vector containing the last 5 output phases from the mission planner
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
    state = {0., 0., 0., 0.};
    dest = {0., 0., 0., 0.};
    phase = {"STANDBY"};
    epsilon = 10;
    status = "STANDBY";
    fovNarrow = 67;
    fovWide = 102;
    theta = -1.0f;
    thetaJoint = 0.0f;
    coarseATime = 0.0;
    coarseBTime = 0.0;
    fineATime = 0.0;
    fineBTime = 0.0;
    jointTime = 0.0f;
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