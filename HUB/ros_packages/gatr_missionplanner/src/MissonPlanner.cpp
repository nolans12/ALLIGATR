#include "headers/MissionPlanner.h"

using namespace std;

MissionPlanner::MissionPlanner() {
    // Constructor
    // drone = uas();
    // env = environment();
}

void MissionPlanner::determine_phase(){
    
}

std::vector<double> MissionPlanner::search(std::vector<double> waypoint) {

    // Check if the drone has identified an RGV
    waypoint = env.get_searchpoint();
    
    //Check to see if the drone has reached the commanded point
    if (check_waypoint_reached(drone.epsilon) == 1){

        // Output that the drone has reached the commanded point
        std::cout << "Destination reached!" << std::endl;

        // Iterate the search location tracker in the environment object
        waypoint = env.next_searchpoint();
    }

    return waypoint;
}

std::vector<double> MissionPlanner::trail(std::vector<double> waypoint) {
    /* Moves toward/follows an RGV
     * Output:
     *         waypoint - 1x3 double vector of commanded point
     */

    // get vectors of all the most recent entries to drone state and RGV positions to minimize calls to back()
    std::vector<double> dronePos, rgvAPos, rgvBPos;
    dronePos = drone.state.back();
    rgvAPos = env.rgvAPosition.back();
    rgvBPos = env.rgvBPosition.back();

    if (env.rgvAInView && env.rgvBInView) {
        // if both RGVs are in view, follow closest one
        if (isRGVAClosest(dronePos, rgvAPos, rgvBPos)) {
            waypoint = (rgvAPos.begin(), rgvAPos.end()-1);
        }
        else {
            waypoint = (rgvBPos.begin(), rgvBPos.end()-1);
        }
    }
    else if (env.rgvAInView) {
        // if RGV-A is in view, follow it
        waypoint = (rgvAPos.begin(), rgvAPos.end()-1);
    }
    else if (env.rgvBInView) {
        // if RGV-B is in view, follow it
        waypoint = (rgvBPos.begin(), rgvBPos.end()-1);
    }
    else if (rgvAPos[3] > rgvBPos[3]) {
        // if no RGV is in view, but most recent rgv detected is RGV-A, target the last known position of it
        target = (rgvAPos.begin(), rgvAPos.end()-1);
    }
    else {
        // if no RGV is in view, but most recent rgv detected is RGV-B, target the last known position of it
        target = (rgvBPos.begin(), rgvBPos.end()-1);
    }

    return waypoint;
}

std::vector<double> MissionPlanner::coarse(std::vector<double> waypoint) {
    /* Circles around an RGV
     * Output:
     *         waypoint - 1x3 double vector of commanded point
     * Makes use of the uas theta iterative property to track where in the orbit
     * the drone is. Makes additional adjustments to the commanded point to
     * avoid exiting the boundary, when necessary. 
     */
    std::vector<double> target, dronePos, rgvAPos, rgvBPos;
    double x, y;
    double thetaStep = 6 * M_PI/180; // change in circle angle over time step [rad]
    int r = 10; // radius of the orbital path [m]
    // get vectors of all the most recent entries to drone state and RGV positions to minimize calls to back()
    dronePos = drone.state.back();
    rgvAPos = env.rgvAPosition.back();
    rgvBPos = env.rgvBPosition.back();

    if (env.rgvAInView && env.rgvBInView) {
        // if both RGVs are in view, set target to the closest one
        if (isRGVAClosest(dronePos, rgvAPos, rgvBPos)) {
            target = (rgvAPos.begin(), rgvAPos.end()-1);
        }
        else {
            target = (rgvBPos.begin(), rgvBPos.end()-1);
        }
    }
    else if (env.rgvAInView) {
        // if RGV-A is in view, target it
        target = (rgvAPos.begin(), rgvAPos.end()-1);
    }
    else if (env.rgvBInView) {
        // if RGV-B is in view, target it
        target = (rgvBPos.begin(), rgvBPos.end()-1);
    }
    else if (rgvAPos[3] > rgvBPos[3]) {
        // if no RGV is in view, but most recent rgv detected is RGV-A, target the last known position of it
        target = (rgvAPos.begin(), rgvAPos.end()-1);
    }
    else {
        // if no RGV is in view, but most recent rgv detected is RGV-B, target the last known position of it
        target = (rgvBPos.begin(), rgvBPos.end()-1);
    }

    // now get the commanded x,y
    if (drone.theta == -1) {
        // if coarse hasn't started yet, initialize the starting angle based
        // on current drone position
        drone.theta = atan2(dronePos[1]-target[1], dronePos[0]-target[0]);
    }
    // increment the angle of the orbit by thetaStep
    drone.theta += thetaStep;
    // set the x and y of the commanded point
    x = target[0] + r*cos(drone.theta);
    y = target[1] + r*sin(drone.theta);

    // if the commanded point would place the drone outside the bounds,
    // change the commanded point to move along the boundary instead
    if (x > (env.bounds[1][0]-1)) {
        x = env.bounds[1][0]-1;
    }
    else if (x < (env.bounds[0][0]+1)) {
        x = env.bounds[0][0]+1;
    }
    if (y > (env.bounds[1][1]-1)) {
        y = env.bounds[1][1]-1;
    }
    else if (y < (env.bounds[0][1]+1)) {
        y = env.bounds[0][1]+1;
    }

    waypoint = {x, y, env.bounds[0][2]};
    return waypoint;
}

std::vector<double> MissionPlanner::fine(std::vector<double> waypoint) {
    /* Hovers directly over an RGV
     * Output:
     *         waypoint - 1x3 double vector of commanded point
     */

    // get vectors of all the most recent entries to drone state and RGV positions to minimize calls to back()
    std::vector<double> dronePos, rgvAPos, rgvBPos;
    dronePos = drone.state.back();
    rgvAPos = env.rgvAPosition.back();
    rgvBPos = env.rgvBPosition.back();

    if (env.rgvAInView && env.rgvBInView) {
        // if both RGVs are in view, follow closest one
        if (isRGVAClosest(dronePos, rgvAPos, rgvBPos)) {
            waypoint = (rgvAPos.begin(), rgvAPos.end()-1);
        }
        else {
            waypoint = (rgvBPos.begin(), rgvBPos.end()-1);
        }
    }
    else if (env.rgvAInView) {
        // if RGV-A is in view, follow it
        waypoint = (rgvAPos.begin(), rgvAPos.end()-1);
    }
    else if (env.rgvBInView) {
        // if RGV-B is in view, follow it
        waypoint = (rgvBPos.begin(), rgvBPos.end()-1);
    }
    // if neither RGV is in view, remain at the same point

    return waypoint;
}

// Makes the drone fly in a square pattern around the environment bounds
std::vector<double> MissionPlanner::bounds_trace(std::vector<double> waypoint){

    //Check to see if the drone has reached the commanded point
    if (check_waypoint_reached(drone.epsilon) == 1){

        //Trace the bounds of the environment
        waypoint = {env.bounds[(drone.p == 1 || drone.p == 2)][0], env.bounds[(drone.p == 2 || drone.p == 3)][1], env.bounds[0][2], 0};

        //If the drone has reached the commanded point, increment the p counter
        drone.p = (drone.p + 1) % 4;
    }

    return waypoint;
}

std::vector<double> MissionPlanner::direct_locate(std::vector<double> waypoint)
{
    // Directly flies to an RGV
    waypoint = {14, 9, 9.144, 0};
    return waypoint;
}

void MissionPlanner::output_drone_state(){
    std::vector<double> dronePos = drone.state.back();
    // Output the current location of the UAS and the next location it is going to
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Current Position: " << dronePos[0] << ", " << dronePos[1] << ", " << dronePos[2] << " -> Going to " << drone.dest[0] << ", " << drone.dest[1] << ", " << drone.dest[2] << std::endl;

}

void MissionPlanner::update_drone_state(std::vector<double> waypoint){
    // Update the state of the UAS
    geometry_msgs::Point state = get_current_location();
    std::vector<double> dronePos = {state.x, state.y, state.z, get_current_heading()};
    /*drone.state[0] = state.x;
    drone.state[1] = state.y;
    drone.state[2] = state.z;
    drone.state[3] = get_current_heading();*/
    drone.state.push_back(dronePos);

    // Update the destination of the UAS
    drone.dest[0] = waypoint[0];
    drone.dest[1] = waypoint[1];
    drone.dest[2] = waypoint[2];
    drone.dest[3] = waypoint[3];
}

bool MissionPlanner::out_of_bounds(std::vector<double> waypoint){
    // Check if a waypoint would fall outside of the environment bounds
    if (waypoint[0] < env.bounds[0][0] || waypoint[0] > env.bounds[1][0] || waypoint[1] < env.bounds[0][1] || waypoint[1] > env.bounds[1][1]){
        return true;
    }
    return false;
}

bool MissionPlanner::RGV_detected(){
    // Check if an RGV is detected through the AR tag node
    return false;
}

bool MissionPlanner::isRGVAClosest(){
        /* Returns true if the closest RGV to the drone is RGV-A
     * Input:  
     *         drone - uas object
     *         env   - enviornment object
     * Output: 
     *         isRGVAClosest - bool true or false
     */

    double distA, distB;
    // get the x-y plane 2d distance from the drone to each RGV
    distA = pow(pow(drone.state[0]-env.rgvAPosition[0], 2) + pow(drone.state[1]-env.rgvAPosition[1], 2), 0.5);
    distB = pow(pow(drone.state[0]-env.rgvBPosition[0], 2) + pow(drone.state[1]-env.rgvBPosition[1], 2), 0.5);

    // return true if the distance from the drone to RGV-A is smaller than the distance to RGV-B
    return (distA <= distB);
}

