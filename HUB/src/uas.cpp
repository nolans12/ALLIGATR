#include "headers/uas.h"
#include "headers/pathStep.h"
#include <string>

uas::uas() {
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
    epsilon = 0.2;
    state[0] = 0.;
    state[1] = 0.;
    state[2] = 0.;
    pathRoot = new pathStep;
    pathNow = pathRoot;
    pathRoot->path[0] = 0.;
    pathRoot->path[1] = 0.;
    pathRoot->path[2] = 0.;
    fovNarrow = 67;
    fovWide = 102;

    p = 0;
    theta = -1.;
    thetaJoint = 0.;
    jointTime = 0.;
    jointComplete = false;
    phaseRoot = new phaseStep;
    phaseNow = phaseRoot;
    phaseRoot->time = 0.;
    phaseRoot->phase = "Takeoff";
    status = "Takeoff";
}