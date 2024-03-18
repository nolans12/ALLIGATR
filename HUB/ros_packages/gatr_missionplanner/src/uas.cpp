#include "headers/uas.h"
//#include "headers/pathStep.h"

uas::uas() {
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
    epsilon = 10;
    state = {0., 0., 0., 0.};
    dest = {0., 0., 0., 0.};
    home = {0.0, 0.0};

    // Camera field of view
    fovNarrow = 47;
    fovWide = 70;

    // Averaging parameters for estimating RGV position
    time_history_average = 10;
    stopped_threshold = 0.1;

    // Can change this to be anything in bounds
    trail_altitude = 35.0 * 0.3048; // 35 ft to m
    coarse_altitude = 35.0 * 0.3048; // 35 ft to m
    fine_altitude = 35.0 * 0.3048; // 35 ft to m

    // Time durations for each phase before moving on to the next phase
    coarse_duration = 30.0; // 10 seconds
    fine_duration = 20.0; // 10 seconds
    joint_duration = 20.0; // 10 seconds
    detection_duration = 2.0; // Time before the RGV is considered lost (s)
    boundary_duration = 5.0; // Time before the UAS aborts when out of bounds (s)
    search_point_duration = 30.0; // Time before the UAS moves on from a search point if it still hasn't reached it(s)
    joint_detection_duration = 15.0; // Time before the UAS aborts when the RGVs are lost in the joint phase (s)

    // If there is no detections within these times, reset the phase timer
    coarse_reset_time = 10.0; // 10 seconds
    fine_reset_time = 10.0; // 10 seconds
    joint_reset_time = 10.0; // 10 seconds

    p = 0;
    theta = -1.0f;
    theta_step = 6 * M_PI/180; // 6 degrees // change in coarse circle angle over time step [rad]
    orbit_radius = 4.0f; // radius of the orbital path [m]
    //thetaJoint = 0.0f;
    jointComplete = false;
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