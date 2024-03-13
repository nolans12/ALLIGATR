#include "headers/MissionPlanner.h"

using namespace std;

MissionPlanner::MissionPlanner() {
    // Constructor
    // drone = uas();
    // env = environment();
}

void MissionPlanner::determine_phase(){
    
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

    waypoint[3] = getYaw(waypoint);

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

    waypoint = {x, y, env.bounds[0][2], getYaw(waypoint);};
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

    waypoint[3] = getYaw(waypoint);

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

    std::vector<double> dronePos = drone.state.back();
    std::vector<double> rgvAPos = env.rgvAPosition.back();
    std::vector<double> rgvBPos = env.rgvBPosition.back();
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

double MissionPlanner::getYaw(std::vector<double> waypoint) {
    /* Get the yaw angle to turn the drone towards the given waypoint
     * Input:
     *         waypoint - 1x4 double vector of commanded point & yaw
     * Output:
     *         yaw - double, yaw angle facing commanded point
     */

    std::vector<double> v = waypoint - drone.state[0];
    yaw = atan2(v[1], v[0]);
    return yaw;
}

std::vector<double> cross(std::vector<double> const &a, std::vector<double> const &b) {
    /* Cross product of 2 1x3 double vectors; a x b
     * Input:
     *         a - 1x3 double vector, left vector in cross product
     *         b - 1x3 double vector, right vector in cross product
     * Output:
     *         cross - 1x3 double vector, result of a x b
     */

    vector<double> r (a.size());  
    r[0] = a[1]*b[2]-a[2]*b[1];
    r[1] = a[2]*b[0]-a[0]*b[2];
    r[2] = a[0]*b[1]-a[1]*b[0];
    return r;
}

double dot(std::vector<double> const &a, std::vector<double> const &b) {
    /* Dot product of 2 1x3 double vectors, a ⋅ b
     * Input:  
     *         a - 1x3 double vector to be dotted
     *         b - 1x3 double vector to be dotted
     * Output:
     *         dot - double, result of a ⋅ b
     */

    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}

double norm(std::vector<double> const &a) {
    /* Get magnitude/norm of arbitrary length double vector
     * Input: 
     *         a - 1d double vector of arbitrary length
     * Output:
     *         norm - double, magnitude of vector a
     */

    double sum = 0;
    for (int i = 0; i < a.size(); i++) {
        sum += pow(a, 2.0);
    }
    return pow(sum, 0.5);
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

// std::vector<float> MissionPlanner::search(std::vector<float> waypoint) {
//     /* 
//      * Returns the next point for a creeping line search pattern
//      * Input:
//      *     drone  -  uas object
//      *     env    -  enviornment object
//      *     waypoint   -  length 4 vector of the previously commanded point and yaw [x, y, z, psi]
//      *     Any units, so long as consistent between passed arguments
//      * Output:
//      *     waypoint   - length 4 vector of the new commanded point and yaw [x, y, z, psi]
//      *     drone  - uas object with p property updated
//      *     env    - enviornment object with boundery property updated
//      * The function makes use of the uas p iterator property to track the
//      * phases of the creeping line search pattern with the following:
//      *     0:    has not begun the search pattern
//      *     1:    is moving to the start point of the search pattern
//      *     even: is moving left to right (along x)
//      *     odd:  is moving up or down (along y)
//      * The value of p is incremented once the drone has entered the search phase, 
//      * and after when it reaches the commanded point for that phase of the search pattern.
//      * The pattern is constructed such that the left/right phases always moves to and from W away from the boundary,
//      * while the up/down phases only go either up or down during a search,
//      * with the up/down length calculated using L and the value of p. 
//      * The upper/lower bounds are calculated from how many Ls fit in the y boundary.
//      */

//     /*
//      *   notes for further implementation:
//      *   add drone, enviornment class file in HUB/src
//      *   env.bounds in format [[x1, y1]; [x2, y2]]
//      *   drone.state in format [x, y, z]
//      *   assumes variables of drone and env are public
//      */


//     // variables always used
//     std::vector<float> fovDims;
//     float L, W, yEnd;
//     short int xneg = 1, yneg = 1, pEnd;

//     //variables used when arranging bounds
//     float d[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
//     short int i, j;
//     short int* indices;
//     float x1_old, y1_old;

//     // length and width of the camera projected box on ground
//     fovDims = drone.getFOVDims();

//     // remove some distance to allow for margin of error as well as RGV moving into last search line from new search line before drone reaches that point
//     // 10% margin can be adjusted as needed
//     L = fovDims[0] * 0.8;
//     W = fovDims[1] * 0.8;

//     // appropriately changes addition to subtraction and vice versa if the boundary has been changed to require it
//     if (env.bounds[1][0] < env.bounds[0][0]) {
//         xneg = -1;
//     }
//     if (env.bounds[1][1] < env.bounds[0][1]) {
//         yneg = -1;
//     }

//     if ((drone.p != 0) && check_waypoint_reached(drone.epsilon) == 1) {
//         // if the drone has reached the commanded point from the previous phase of search, increase the iteration counter p by 1
//         drone.p++;
//     }

//     if (drone.p == 0) {
//         /* if the drone has not entered the search pattern yet,
//          * first change the boundary variables such that x1,y1 is the closest boundary corner to the drone, 
//          * then move to the start location for the creeping line search pattern
//          */
//         for (i = 0; i < 2; i++) {
//             for (j = 0; j < 2; j++) {
//                 // gets the distance from the current drone location to each corner
//                 d[i][j] = sqrt(pow(env.bounds[i][0]-drone.state[0], 2) + pow(env.bounds[j][1]-drone.state[1], 2));
//             }
//         }
//         // gets the indices of the closest corner
//         indices = minInd(d);
//         i = indices[0];
//         j = indices[1];
//         // check if bounds are same first to save some computation time
//         if ((i != 0) || (j != 0)){
//             // change x1,y1 to the closest corner then change x2,y2 accordingly
//             x1_old = env.bounds[0][0];
//             y1_old = env.bounds[0][1];
//             env.bounds[0][0] = env.bounds[i][0];
//             env.bounds[0][1] = env.bounds[j][1];
//             if (i == 1) {
//                 env.bounds[1][0] = x1_old;
//             }
//             if (j == 1) {
//                 env.bounds[1][1] = y1_old;
//             }
//             if (env.bounds[1][0] < env.bounds[0][0]) {
//                 xneg = -1;
//             }
//             if (env.bounds[1][1] < env.bounds[0][1]) {
//                 yneg = -1;
//             }
//             // set commanded point to the closest corner + camera buffer
//             waypoint[0] = env.bounds[0][0]+xneg*W;
//             waypoint[1] = env.bounds[0][1]+yneg*L;
//             waypoint[2] = drone.state[3];
//             drone.p++;
//         }
//     }
//     else if ((drone.p % 2) == 0) {
//         // if p is even, the drone is on left-right portion of flight
//         if (abs(drone.state[0]-(env.bounds[0][0]+xneg*W)) < 0.2) {
//             // if drone is on the left, set commanded point to the right
//             waypoint[0] = env.bounds[0][1]-xneg*W;
//             waypoint[1] = drone.state[1];
//             waypoint[2] = drone.state[2];
//             waypoint[3] = 0;
//         }
//         else if (abs(drone.state[0]-(env.bounds[1][0]+xneg*W)) < 0.2) {
//             // if drone is on the right, set commanded point to the left
//             waypoint[0] = env.bounds[0][0]+xneg*W;
//             waypoint[1] = drone.state[1];
//             waypoint[2] = drone.state[2];
//             waypoint[3] = 0;
//         }
//     }
//     else if (((drone.p % 2) == 1) && (drone.p != 1)) {
//         // if p is odd, the drone is on upwards portion of flight
//         // ignore when p is 1 - moving to search start point
//         waypoint[0] = drone.state[0];
//         waypoint[1] = env.bounds[0][1]+yneg*L*drone.p;
//         waypoint[2] = drone.state[2];
//         waypoint[3] = 90;
//     }

//     // get the p and y of the last phase we want
//     pEnd = floor(abs(env.bounds[1][1]-env.bounds[0][1])/L);
//     yEnd = env.bounds[0][1] + yneg*L*pEnd;
//     if ((pEnd%2) != 0) {
//         pEnd++;
//     }
//     if ((drone.p > pEnd) || (((yneg == 1) && (drone.state[1] >= yEnd)) || ((yneg == -1) && (drone.state[1] <= yEnd)))) {
//         // if we are outside the last p and y we want, set p = 0, restarting search
//         drone.p = 0;
//     }
//     return waypoint;
// }