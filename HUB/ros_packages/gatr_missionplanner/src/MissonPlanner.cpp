#include "headers/MissionPlanner.h"

using namespace std;

MissionPlanner::MissionPlanner() {
    // Constructor
    // drone = uas();
    // env = environment();
    phase = "Search";
    phases = {phase};
    smootherCount = 0;
}

MissionPlanner::MissionPlanner(ros::NodeHandle gnc_node) {
    // Constructor
    // drone = uas();
    // env = environment();
    phase = "Search";
    phases = {phase};
    smootherCount = 0;
    rel_coord_A_sub = gnc_node.subscribe("CV/rel_coord_A", 1, &MissionPlanner::rgvA_detected_callback, this);
}

///////////// Phases //////////////////////
void MissionPlanner::determine_phase(){
    if (phase == "Boundary Control") {
        boundary_control_phase();
    } else if (phase == "Search") {
        search_phase();
    } else if (phase == "Trail") {
        trail_phase();
    } else if (phase == "Coarse") {
        coarse_phase();
    } else if (phase == "Fine") {
        fine_phase();
    } else if (phase == "Joint") {
        joint_phase();
    } else {
        ROS_ERROR("!!! ABORTING MISSION !!! - Invalid phase passed to motion planner!");
        phase = "ABORT";
    }
}

void MissionPlanner::boundary_control_phase(){
    
    // Is the drone out of bounds?
    if (out_of_bounds(drone.state)){
        phase = "Boundary Control";
    }

    // If the drone is not out of bounds, move to the search phase
    else{
        phase = "Search";
    }
}

void MissionPlanner::search_phase(){
    if (out_of_bounds(drone.state)){
        ROS_INFO("Drone is out of bounds. Moving to boundary control phase...");
        phase = "Boundary Control";
    }

    // Check to see if the CV algorithm has detected an RGV
    else if (RGV_detected()){

        // RGV A is detected
        if (env.rgvAInView){

            // Has RGV A been finely localized?
            if (env.rgvAFineComplete){

                // Has RGV B been finely localized as well?
                if (env.rgvBFineComplete){
                    // If both RGVs have been finely localized, move to the joint phase
                    phase = "Joint";
                    ROS_INFO("Both RGVs have been localized. Moving to joint phase...");
                }

                else{
                    // If RGV A has been finely localized, but RGV B has not, keep looking for RGV B
                    phase = "Search";
                    ROS_INFO("RGV A has been localized, but RGV B has not. Continuing search...");
                }

            }

            else if (env.rgvACoarseComplete){
                // If RGV A has been coarsely localized, but not finely localized, move to the fine phase
                phase = "Fine";
                ROS_INFO("RGV A has been coarsely localized. Moving to fine phase...");
            }

            else{
                // If RGV A has been detected but not been coarsely localized, start trailing it
                phase = "Trail";
                ROS_INFO("RGV A has been detected but not localized. Starting trail phase...");
            }   
        } 

        // RGV B is detected
        else if (env.rgvBInView){

            // Has RGV B been finely localized?
            if (env.rgvBFineComplete){

                // Has RGV A been finely localized as well?
                if (env.rgvAFineComplete){
                    // If both RGVs have been finely localized, move to the joint phase
                    phase = "Joint";
                    ROS_INFO("Both RGVs have been localized. Moving to joint phase...");
                }

                else{
                    // If RGV B has been finely localized, but RGV A has not, keep looking for RGV A
                    phase = "Search";
                    ROS_INFO("RGV B has been localized, but RGV A has not. Continuing search...");
                }

            }

            else if (env.rgvBCoarseComplete){
                // If RGV B has been coarsely localized, but not finely localized, move to the fine phase
                phase = "Fine";
                ROS_INFO("RGV B has been coarsely localized. Moving to fine phase...");
            }

            else{
                // If RGV B has been detected but not been coarsely localized, start trailing it
                phase = "Trail";
                ROS_INFO("RGV B has been detected but not localized. Starting trail phase...");
            }   

        }
    }
    
    else{
        // If no RGVs have been detected, keep searching
        phase = "Search";
    }

    
}

void MissionPlanner::trail_phase(){
    return;
}

void MissionPlanner::coarse_phase(){
    return;
}

void MissionPlanner::fine_phase(){
    return;
}

void MissionPlanner::joint_phase(){
    return;
}

///////////// Motions //////////////////////
std::vector<double> MissionPlanner::determine_motion(std::vector<double> waypoint){
    if (phase == "Boundary Control") {
        return boundary_control_motion(waypoint);
    } else if (phase == "Search") {
        return search_motion(waypoint);
    } else if (phase == "Trail") {
        return trail_motion(waypoint);
    } else if (phase == "Coarse") {
        return coarse_motion(waypoint);
    } else if (phase == "Fine") {
        return fine_motion(waypoint);
    } else if (phase == "Joint") {
        return joint_motion(waypoint);
    } else if (phase == "ABORT") {
        ROS_INFO("Landing drone...");
    } else {
        ROS_ERROR("!!! ABORTING MISSION !!! - Invalid phase passed to motion planner!");
        phase = "ABORT";
    }
}

std::vector<double> MissionPlanner::boundary_control_motion(std::vector<double> waypoint) {
    /* Set waypoint to middle of mission boundary
     * Input:
     *         waypoint - 1x4 double vector of previously commanded point & yaw
     * Output:
     *         waypoint - 1x4 double vector of commanded point & yaw
     */
    waypoint[0] = (env.bounds[0][0]+env.bounds[1][0])/2.0;
    waypoint[1] = (env.bounds[0][1]+env.bounds[1][1])/2.0;
    waypoint[2] = (env.bounds[0][2]+env.bounds[1][2])/2.0;
    waypoint[3] = getYaw(waypoint);
    return waypoint;
}

std::vector<double> MissionPlanner::search_motion(std::vector<double> waypoint) {
    /* Follows lawnmower patter around bounds 
     * Input:
     *         waypoint - 1x4 double vector of previously commanded point & yaw
     * Output:
     *         waypoint - 1x4 double vector of commanded point & yaw
     */

    // Check if the drone has identified an RGV
    waypoint = env.get_searchpoint();
    
    //Check to see if the drone has reached the commanded point
    if (check_waypoint_reached(drone.epsilon) == 1){

        // Output that the drone has reached the commanded point
        std::cout << "Destination reached!" << std::endl;

        // Iterate the search location tracker in the environment object
        waypoint = env.next_searchpoint();
    }

    waypoint[3] = getYaw(waypoint);

    return waypoint;
}

std::vector<double> MissionPlanner::trail_motion(std::vector<double> waypoint) {
    /* Moves toward/follows an RGV
     * Input:
     *         waypoint - 1x4 double vector of previously commanded point & yaw
     * Output:
     *         waypoint - 1x4 double vector of commanded point & yaw
     */

    // get vectors of all the most recent entries to drone state and RGV positions to minimize calls to back()
    // std::vector<double> dronePos, rgvAPos, rgvBPos;
    // dronePos = drone.state.back();
    // rgvAPos = env.rgvAPosition;
    // rgvBPos = env.rgvBPosition;

    if (env.rgvAInView && env.rgvBInView) {
        // if both RGVs are in view, follow closest one
        if (isRGVAClosest()) {
            waypoint[0] = env.rgvAPosition[0];
            waypoint[1] = env.rgvAPosition[1];
        }
        else {
            waypoint[0] = env.rgvBPosition[0];
            waypoint[1] = env.rgvBPosition[1];
        }
    }
    else if (env.rgvAInView) {
        // if RGV-A is in view, follow it
        waypoint[0] = env.rgvAPosition[0];
        waypoint[1] = env.rgvAPosition[1];
    }
    else if (env.rgvBInView) {
        // if RGV-B is in view, follow it
        waypoint[0] = env.rgvBPosition[0];
        waypoint[1] = env.rgvBPosition[1];
    }
    else if (env.rgvAPosition[3] > env.rgvBPosition[3]) {
        // if no RGV is in view, but most recent rgv detected is RGV-A, target the last known position of it
        waypoint[0] = env.rgvAPosition[0];
        waypoint[1] = env.rgvAPosition[1];
    }
    else {
        // if no RGV is in view, but most recent rgv detected is RGV-B, target the last known position of it
        waypoint[0] = env.rgvBPosition[0];
        waypoint[1] = env.rgvBPosition[1];
    }

    waypoint[3] = getYaw(waypoint);
    waypoint[2] = drone.trail_altitude;

    return waypoint;
}

std::vector<double> MissionPlanner::coarse_motion(std::vector<double> waypoint) {
    /* Circles around an RGV
     * Input:
     *         waypoint - 1x4 double vector of previously commanded point & yaw
     * Output:
     *         waypoint - 1x4 double vector of commanded point & yaw
     * 
     * Makes use of the uas theta iterative property to track where in the orbit
     * the drone is. Makes additional adjustments to the commanded point to
     * avoid exiting the boundary, when necessary. 
     */

    std::vector<double> target;
    double x, y;
    // double thetaStep = 6 * M_PI/180; // change in circle angle over time step [rad]
    // int r = 10; // radius of the orbital path [m]
    // get vectors of all the most recent entries to drone state and RGV positions to minimize calls to back()
    // dronePos = drone.state.back();
    // rgvAPos = env.rgvAPosition.back();
    // rgvBPos = env.rgvBPosition.back();

    if (env.rgvAInView && env.rgvBInView) {
        // if both RGVs are in view, set target to the closest one
        if (isRGVAClosest()) {
            target = {env.rgvAPosition[0], env.rgvAPosition[1]};
        }
        else {
            target = {env.rgvBPosition[0], env.rgvBPosition[1]};
        }
    }
    else if (env.rgvAInView) {
        // if RGV-A is in view, target it
        target = {env.rgvAPosition[0], env.rgvAPosition[1]};
    }
    else if (env.rgvBInView) {
        // if RGV-B is in view, target it
        target = {env.rgvBPosition[0], env.rgvBPosition[1]};
    }
    else if (env.rgvAPosition[3] > env.rgvBPosition[3]) {
        // if no RGV is in view, but most recent rgv detected is RGV-A, target the last known position of it
        target = {env.rgvAPosition[0], env.rgvAPosition[1]};
    }
    else {
        // if no RGV is in view, but most recent rgv detected is RGV-B, target the last known position of it
        target = {env.rgvBPosition[0], env.rgvBPosition[1]};
    }

    // now get the commanded x,y
    if (drone.theta == -1) {
        // if coarse hasn't started yet, initialize the starting angle based
        // on current drone position
        drone.theta = atan2(drone.state[1]-target[1], drone.state[0]-target[0]);
    }
    // increment the angle of the orbit by thetaStep
    drone.theta += drone.theta_step;
    // set the x and y of the commanded point
    x = target[0] + drone.orbit_radius*cos(drone.theta);
    y = target[1] + drone.orbit_radius*sin(drone.theta);

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

    waypoint = {x, y, drone.coarse_altitude, getYaw(waypoint)};
    return waypoint;
}

std::vector<double> MissionPlanner::fine_motion(std::vector<double> waypoint) {
    /* Hovers directly over an RGV
     * Input:
     *         waypoint - 1x4 double vector of previously commanded point & yaw
     * Output:
     *         waypoint - 1x4 double vector of commanded point & yaw
     */

    // get vectors of all the most recent entries to drone state and RGV positions to minimize calls to back()
    // std::vector<double> dronePos, rgvAPos, rgvBPos;
    // dronePos = drone.state.back();
    // rgvAPos = env.rgvAPosition.back();
    // rgvBPos = env.rgvBPosition.back();

    if (env.rgvAInView && env.rgvBInView) {
        // if both RGVs are in view, follow closest one
        if (isRGVAClosest()) {
            waypoint[0] = env.rgvAPosition[0];
            waypoint[1] = env.rgvAPosition[1];
        }
        else {
            waypoint[0] = env.rgvBPosition[0];
            waypoint[1] = env.rgvBPosition[1];
        }
    }
    else if (env.rgvAInView) {
        // if RGV-A is in view, follow it
        waypoint[0] = env.rgvAPosition[0];
        waypoint[1] = env.rgvAPosition[1];
    }
    else if (env.rgvBInView) {
        // if RGV-B is in view, follow it
            waypoint[0] = env.rgvBPosition[0];
            waypoint[1] = env.rgvBPosition[1];
    }
    // if neither RGV is in view, remain at the same point

    waypoint[3] = getYaw(waypoint);
    waypoint[2] = drone.fine_altitude;

    return waypoint;
}

std::vector<double> MissionPlanner::joint_motion(std::vector<double> waypoint) {
    /* Goes to middle point of most recent RGV positions, then flies to either:
     * minimum height to have both RGVs in view, or
     * max height in bounds and slowly spins to find both RGVs
     * Input:
     *         waypoint - 1x4 double vector of previously commanded point & yaw
     * Output:
     *         waypoint - 1x4 double vector of commanded point & yaw
     */

    std::vector<double> dronePos = drone.state;
    std::vector<double> rgvAPos = env.rgvAPosition;
    std::vector<double> rgvBPos = env.rgvBPosition;
    std::vector<double> v1, v2, crossed;
    double midpoint[2] = {(rgvAPos[0]+rgvBPos[0])/2, (rgvAPos[1]+rgvBPos[1])/2};
    double width, desiredHeight, yaw;

    if (env.rgvAInView && env.rgvBInView) {
        // if both RGVs are in view, optimize z height
        width = norm((rgvAPos.begin(), rgvAPos.end()-1) - (rgvBPos.begin(), rgvBPos.end()-1)) / 2.0 + drone.fovWide*0.1;
        desiredHeight = width / tan(drone.fovWide*M_PI/360.0);
        if (desiredHeight < env.bounds[1][2]) {
            desiredHeight = env.bounds[1][2];
        }

        if (rgvAPos[1] < rgvBPos[1]) {
            v1 = {rgvAPos[0]-rgvBPos[0], rgvAPos[1]-rgvBPos[1], 0};
        }
        else {
            v1 = {rgvBPos[0]-rgvAPos[0], rgvBPos[1]-rgvAPos[1], 0};
        }
        v2 = {1, 0, 0};
        yaw = atan2(norm(cross(v1,v2)), dot(v1,v2)) - M_PI;

        waypoint = {(rgvAPos[0]+rgvBPos[0])/2, (rgvAPos[1]+rgvBPos[1])/2, desiredHeight, yaw};
    }
    else {
        waypoint = {(rgvAPos[0]+rgvBPos[0])/2, (rgvAPos[1]+rgvBPos[1])/2, env.bounds[1][2], waypoint[3]};
    }

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

    waypoint[3] = getYaw(waypoint);

    return waypoint;
}

std::vector<double> MissionPlanner::direct_locate(std::vector<double> waypoint)
{
    // Directly flies to an RGV
    waypoint = {14, 9, 9.144, 0};
    return waypoint;
}

///////////// Computer Vision + ROS ///////////////////
// Callback function to be updated when the RGV is detected
void MissionPlanner::rgvA_detected_callback(const std_msgs::Float32MultiArray::ConstPtr& coords)
{
	// Display coords
	ROS_INFO("RGV Coords: [%f, %f]", coords->data[0], coords->data[1]);

	// Check if the RGV is in view
    if(coords->data[0]*10000 !=0.0 || coords->data[1]*10000 != 0.0){
		env.rgvAInView = true;
	}
	else{
		env.rgvAInView = false;
	}
    
}


///////////// Helper Functions //////////////////////
double MissionPlanner::getYaw(std::vector<double> waypoint) {
    /* Get the yaw angle to turn the drone towards the given waypoint
     * Input:
     *         waypoint - 1x4 double vector of commanded point & yaw
     * Output:
     *         yaw - double, yaw angle facing commanded point
     */

    //std::vector<double> v = waypoint - drone.state[0];
    double yaw = atan2(waypoint[0] - drone.state[0], waypoint[1] - drone.state[1]);
    return yaw;
}



void MissionPlanner::output_drone_state(){
    std::vector<double> dronePos = drone.state;
    // Output the current location of the UAS and the next location it is going to
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Current Position: " << dronePos[0] << ", " << dronePos[1] << ", " << dronePos[2] << " -> Going to " << drone.dest[0] << ", " << drone.dest[1] << ", " << drone.dest[2] << std::endl;

}

void MissionPlanner::update_drone_state(std::vector<double> waypoint){
    // Update the state of the UAS
    geometry_msgs::Point state = get_current_location();
    std::vector<double> dronePos = {state.x, state.y, state.z, get_current_heading()};
    drone.state[0] = state.x;
    drone.state[1] = state.y;
    drone.state[2] = state.z;
    drone.state[3] = get_current_heading();
    //drone.state.push_back(dronePos);

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

std::string MissionPlanner::getPhase(){
    return phase;
}

void MissionPlanner::setPhase(std::string phaseIn){
    phase = phaseIn;
}

