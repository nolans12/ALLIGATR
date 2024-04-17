#include "headers/MissionPlanner.h"

using namespace std;

MissionPlanner::MissionPlanner() {
    // Constructor
    // drone = uas();
    // env = environment();
    phase = "Search";
    search_point_time = ros::Time::now();
    phases = {phase};
    smootherCount = 0;
}

MissionPlanner::MissionPlanner(ros::NodeHandle gnc_node) {
    // Constructor
    // drone = uas();
    // env = environment();
    phase = "Search";
    thought = "Searching";
    target_lock = "None";
    search_point_time = ros::Time::now();
    phases = {phase};
    smootherCount = 0;
    inert_coord_A_sub_primary = gnc_node.subscribe("CV/inert_coord_A", 1, &MissionPlanner::rgvA_detected_callback, this);
    inert_coord_B_sub_primary = gnc_node.subscribe("CV/inert_coord_B", 1, &MissionPlanner::rgvB_detected_callback, this);
    inert_coord_A_sub_secondary = gnc_node.subscribe("CV/Secondary/inert_coord_A", 1, &MissionPlanner::rgvA_detected_callback_s, this);
    inert_coord_B_sub_secondary = gnc_node.subscribe("CV/Secondary/inert_coord_B", 1, &MissionPlanner::rgvB_detected_callback_s, this);
    uas_state_sub = gnc_node.subscribe("mavros/local_position/pose", 3, &MissionPlanner::get_current_location_mav, this);
    phase_pub = gnc_node.advertise<std_msgs::String>("MP/phase", 10);
    speak_pub = gnc_node.advertise<std_msgs::String>("MP/speak", 10);

}

MissionPlanner::~MissionPlanner() {
    // Close CSV file
    // rgvA_csv.close();
    // rgvB_csv.close();
    // uas_csv.close();
    // uas_csv_rgvA.close();
    // uas_csv_rgvB.close();

}

///////////// Computer Vision + ROS ///////////////////
// Callback function to be updated when the RGV is detected
void MissionPlanner::rgvA_detected_callback(const std_msgs::Float32MultiArray::ConstPtr& coords)
{
    if (primary_activated){
        env.rgvAInView = true;
        last_rgvA_detection = ros::Time::now();
        std::vector<double> rgvA_coords = {coords->data[0], coords->data[1], 0.0};
        //std::vector<double> local_rgvA_coords = enu_2_local(rgvA_coords);
        std::vector<double> local_rgvA_coords = rgvA_coords;
        double x = local_rgvA_coords[0];
        double y = local_rgvA_coords[1];
        // env.rgvAPosition[0] = x;
        // env.rgvAPosition[1] = y;

        // Update the history of RGV A positions
        env.rgvAHistory.x_pos.push_back(x);
        env.rgvAHistory.y_pos.push_back(y);
        env.rgvAHistory.time.push_back(last_rgvA_detection.toSec());

        // If the history of RGV A positions is longer than the time history average, pop the oldest entry
        if (env.rgvAHistory.x_pos.size() > drone.time_history_average){
            env.rgvAHistory.x_pos.erase(env.rgvAHistory.x_pos.begin());
            env.rgvAHistory.y_pos.erase(env.rgvAHistory.y_pos.begin());  
            env.rgvAHistory.time.erase(env.rgvAHistory.time.begin());
        }

        // set env.rgvAPosition to the average of all the positions in the history
        env.rgvAPosition[0] = std::accumulate(env.rgvAHistory.x_pos.begin(), env.rgvAHistory.x_pos.end(), 0.0) / env.rgvAHistory.x_pos.size();
        env.rgvAPosition[1] = std::accumulate(env.rgvAHistory.y_pos.begin(), env.rgvAHistory.y_pos.end(), 0.0) / env.rgvAHistory.y_pos.size();
    } 
}

// Callback function to be updated when the RGV is detected
void MissionPlanner::rgvB_detected_callback(const std_msgs::Float32MultiArray::ConstPtr& coords)
{
    if (primary_activated){
        env.rgvBInView = true;
        last_rgvB_detection = ros::Time::now();
        std::vector<double> rgvB_coords = {coords->data[0], coords->data[1], 0};
        //std::vector<double> local_rgvB_coords = enu_2_local(rgvB_coords);
        std::vector<double> local_rgvB_coords = rgvB_coords;
        double x = local_rgvB_coords[0];
        double y = local_rgvB_coords[1];

        // Update the history of RGV B positions
        env.rgvBHistory.x_pos.push_back(x);
        env.rgvBHistory.y_pos.push_back(y);
        env.rgvBHistory.time.push_back(last_rgvB_detection.toSec());

        // If the history of RGV B positions is longer than the time history average, pop the oldest entry
        if (env.rgvBHistory.x_pos.size() > drone.time_history_average){
            env.rgvBHistory.x_pos.erase(env.rgvBHistory.x_pos.begin());
            env.rgvBHistory.y_pos.erase(env.rgvBHistory.y_pos.begin());  
            env.rgvBHistory.time.erase(env.rgvBHistory.time.begin());
        }

        // set env.rgvBPosition to the average of all the positions in the history
        env.rgvBPosition[0] = std::accumulate(env.rgvBHistory.x_pos.begin(), env.rgvBHistory.x_pos.end(), 0.0) / env.rgvBHistory.x_pos.size();
        env.rgvBPosition[1] = std::accumulate(env.rgvBHistory.y_pos.begin(), env.rgvBHistory.y_pos.end(), 0.0) / env.rgvBHistory.y_pos.size();
    }   

}

// Callback function to be updated when the RGV is detected
void MissionPlanner::rgvA_detected_callback_s(const std_msgs::Float32MultiArray::ConstPtr& coords)
{
    if (secondary_activated){
        env.rgvAInView = true;
        last_rgvA_detection = ros::Time::now();
        std::vector<double> rgvA_coords = {coords->data[0], coords->data[1], 0.0};
        //std::vector<double> local_rgvA_coords = enu_2_local(rgvA_coords);
        std::vector<double> local_rgvA_coords = rgvA_coords;
        double x = local_rgvA_coords[0];
        double y = local_rgvA_coords[1];
        // env.rgvAPosition[0] = x;
        // env.rgvAPosition[1] = y;

        // Update the history of RGV A positions
        env.rgvAHistory.x_pos.push_back(x);
        env.rgvAHistory.y_pos.push_back(y);
        env.rgvAHistory.time.push_back(last_rgvA_detection.toSec());

        // If the history of RGV A positions is longer than the time history average, pop the oldest entry
        if (env.rgvAHistory.x_pos.size() > drone.time_history_average){
            env.rgvAHistory.x_pos.erase(env.rgvAHistory.x_pos.begin());
            env.rgvAHistory.y_pos.erase(env.rgvAHistory.y_pos.begin());  
            env.rgvAHistory.time.erase(env.rgvAHistory.time.begin());
        }

        // set env.rgvAPosition to the average of all the positions in the history
        env.rgvAPosition[0] = std::accumulate(env.rgvAHistory.x_pos.begin(), env.rgvAHistory.x_pos.end(), 0.0) / env.rgvAHistory.x_pos.size();
        env.rgvAPosition[1] = std::accumulate(env.rgvAHistory.y_pos.begin(), env.rgvAHistory.y_pos.end(), 0.0) / env.rgvAHistory.y_pos.size();
    } 
}

// Callback function to be updated when the RGV is detected
void MissionPlanner::rgvB_detected_callback_s(const std_msgs::Float32MultiArray::ConstPtr& coords)
{
    if (secondary_activated){
        env.rgvBInView = true;
        last_rgvB_detection = ros::Time::now();
        std::vector<double> rgvB_coords = {coords->data[0], coords->data[1], 0};
        //std::vector<double> local_rgvB_coords = enu_2_local(rgvB_coords);
        std::vector<double> local_rgvB_coords = rgvB_coords;
        double x = local_rgvB_coords[0];
        double y = local_rgvB_coords[1];

        // Update the history of RGV B positions
        env.rgvBHistory.x_pos.push_back(x);
        env.rgvBHistory.y_pos.push_back(y);
        env.rgvBHistory.time.push_back(last_rgvB_detection.toSec());

        // If the history of RGV B positions is longer than the time history average, pop the oldest entry
        if (env.rgvBHistory.x_pos.size() > drone.time_history_average){
            env.rgvBHistory.x_pos.erase(env.rgvBHistory.x_pos.begin());
            env.rgvBHistory.y_pos.erase(env.rgvBHistory.y_pos.begin());  
            env.rgvBHistory.time.erase(env.rgvBHistory.time.begin());
        }

        // set env.rgvBPosition to the average of all the positions in the history
        env.rgvBPosition[0] = std::accumulate(env.rgvBHistory.x_pos.begin(), env.rgvBHistory.x_pos.end(), 0.0) / env.rgvBHistory.x_pos.size();
        env.rgvBPosition[1] = std::accumulate(env.rgvBHistory.y_pos.begin(), env.rgvBHistory.y_pos.end(), 0.0) / env.rgvBHistory.y_pos.size();
    }   

}

void MissionPlanner::get_current_location_mav(const geometry_msgs::PoseStamped::ConstPtr& msg){
    // Update the state of the UAS in the local frame as defined in the mavros node
    drone.state = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, get_current_heading()};
    //drone.vel = {msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z};

    //output_drone_state();
}

void MissionPlanner::update_drone_state(std::vector<double> waypoint){
    // Update the state of the UAS
    //geometry_msgs::Point state = get_current_location_mav();
    //drone.state = {state.x, state.y, state.z, get_current_heading()};

    // Update the destination of the UAS
    //std::vector<double> local_waypoint = enu_2_local(waypoint);
    drone.dest = {waypoint[0], waypoint[1], waypoint[2], waypoint[3]};

    // Publish the phase of the UAS
    std_msgs::String phase_msg;
    std_msgs::String speak_msg;
    phase_msg.data = phase;
    speak_msg.data = thought;
    phase_pub.publish(phase_msg);
    // say(thought);

    // Check when the last time the rgvs were seen and if they are still in view
    if (env.rgvAInView)
    {
        if (ros::Time::now() - last_rgvA_detection > ros::Duration(drone.detection_duration)){
            env.rgvAInView = false;

            // Clear the history of the RGV's position
            env.rgvAHistory.x_pos.clear();
            env.rgvAHistory.y_pos.clear();
            env.rgvAHistory.time.clear();
        }
    }

    if (env.rgvBInView)
    {
        if (ros::Time::now() - last_rgvB_detection > ros::Duration(drone.detection_duration)){
            env.rgvBInView = false;

            // Clear the history of the RGV's position
            env.rgvBHistory.x_pos.clear();
            env.rgvBHistory.y_pos.clear();
            env.rgvBHistory.time.clear();
        }
    }

    // Reset the time in coarse localization timers if it has been too long since an RGV was seen
    if (time_coarsely_localized > ros::Duration(0.0)) {
        // check if the time since the last RGV was seen is greater than the coarse duration
        if (ros::Time::now() - last_rgvA_detection > ros::Duration(drone.coarse_reset_time) && ros::Time::now() - last_rgvB_detection > ros::Duration(drone.coarse_reset_time)){
            time_coarsely_localized = ros::Duration(0.0);
        }
    }

    // Reset the time in fine localization timers if it has been too long since an RGV was seen
    if (time_finely_localized > ros::Duration(0.0)) {
        // check if the time since the last RGV was seen is greater than the fine duration
        if (ros::Time::now() - last_rgvA_detection > ros::Duration(drone.fine_reset_time) && ros::Time::now() - last_rgvB_detection > ros::Duration(drone.fine_reset_time)){
            time_finely_localized = ros::Duration(0.0);
        }
    }

    // Reset the time in joint localization timers if it has been too long since an RGV was seen
    if (time_joint_localized > ros::Duration(0.0)) {
        // check if the time since the last RGV was seen is greater than the joint duration
        if (ros::Time::now() - last_rgvA_detection > ros::Duration(drone.joint_reset_time) || ros::Time::now() - last_rgvB_detection > ros::Duration(drone.joint_reset_time)){
            time_joint_localized = ros::Duration(0.0);
        }
    }
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
    } else if (phase == "Joint Search") {
        joint_search_phase();
    } else if (phase == "Return Home") {
        return_home_phase();
    } else if (phase == "ABORT") {
        ROS_INFO("Landing drone...");
    }
    else {
        ROS_ERROR("!!! ABORTING MISSION !!! - Invalid phase passed to motion planner!");
        phase = "ABORT";
    }
}

void MissionPlanner::boundary_control_phase(){
    
    //////// OVERRIDE ////////

    // phase = "Search";
    // search_point_time = ros::Time::now();
    // ROS_INFO("Boundary control phase overridden. Search phase sent instead...");

    //////////////////////////

    // Is the drone out of bounds?
    if (out_of_bounds(drone.state)){
        phase = "Boundary Control";
    }

    // else if(ros::Time::now() - out_of_bounds_time > ros::Duration(drone.boundary_duration)){
    //     // If the drone has been within bounds for more than X seconds, move to the search phase
    //     phase = "ABORT";
    //     ROS_FATAL("Drone has been within bounds for %f seconds. ABORTING", (ros::Time::now() - out_of_bounds_time).toSec());
    // }

    // If the drone is not out of bounds, move to the search phase
    else{
        ROS_INFO("Drone is back within bounds. Moving to search phase...");
        phase = "Search";
        search_point_time = ros::Time::now();
    }
}

void MissionPlanner::return_home_phase(){
    // If the drone is within 10 meters of the home position, land
    if (norm({drone.state[0] - drone.home[0], drone.state[1] - drone.home[1]}) < 1){
        phase = "ABORT";
        ROS_INFO("Drone is within 10 meters of home. Landing drone...");
        thought = "Landing";
        say(thought);
    }
}

void MissionPlanner::search_phase(){

    // Make sure the correct cameras are activated
    set_cameras(true, true);

    if (out_of_bounds(drone.state)){
        ROS_INFO("Drone is out of bounds. Moving to boundary control phase...");
        phase = "Boundary Control";
        out_of_bounds_time = ros::Time::now();
    }

    // RGV A is detected
    if (env.rgvAInView){

        // Has RGV A been finely localized?
        if (env.rgvAFineComplete){

            // Has RGV B been finely localized as well?
            if (env.rgvBFineComplete){
                // If both RGVs have been finely localized, move to the joint phase
                phase = "Joint";
                joint_time_engaged = ros::Time::now();
                joint_last_detection = ros::Time::now();
                ROS_INFO("Both RGVs have been localized. Moving to joint phase...");
                thought = "Joint";
                say(thought);

                return;
            }

            else{
                // If RGV A has been finely localized, but RGV B has not, keep looking for RGV B
                phase = "Search";
                ROS_INFO("RGV A has been localized, but RGV B has not. Continuing search...");
                //thought = "Localized that one already";
            }

        }

        else if (env.rgvACoarseComplete){
            // If RGV A has been coarsely localized, but not finely localized, move to the fine phase
            phase = "Fine";
            fine_time_engaged = ros::Time::now();
            ROS_INFO("RGV A has been coarsely localized. Moving to fine phase...");
            thought = "Coarsely localized. Moving to fine phase";
            say(thought);
            return;
        }

        else{
            // If RGV A has been detected but not been coarsely localized, start trailing it
            phase = "Trail";
            ROS_INFO("RGV A has been detected but not localized. Starting trail phase...");
            thought = "Trailing";
            say(thought);
            return;
        }   
    } 

    // RGV B is detected
    if (env.rgvBInView){

        // Has RGV B been finely localized?
        if (env.rgvBFineComplete){

            // Has RGV A been finely localized as well?
            if (env.rgvAFineComplete){
                // If both RGVs have been finely localized, move to the joint phase
                phase = "Joint";
                joint_time_engaged = ros::Time::now();
                joint_last_detection = ros::Time::now();
                ROS_INFO("Both RGVs have been localized. Moving to joint phase...");
                thought = "Joint";
                say(thought);
                return;
            }

            else{
                // If RGV B has been finely localized, but RGV A has not, keep looking for RGV A
                phase = "Search";
                ROS_INFO("RGV B has been localized, but RGV A has not. Continuing search...");
                //thought = "Localized that one already";
            }

        }

        else if (env.rgvBCoarseComplete){
            // If RGV B has been coarsely localized, but not finely localized, move to the fine phase
            phase = "Fine";
            fine_time_engaged = ros::Time::now();
            ROS_INFO("RGV B has been coarsely localized. Moving to fine phase...");
            thought = "Coarsely localized. Moving to fine phase";
            say(thought);
            return;
        }

        else{
            // If RGV B has been detected but not been coarsely localized, start trailing it
            phase = "Trail";
            ROS_INFO("RGV B has been detected but not localized. Starting trail phase...");
            thought = "Trailing";
            say(thought);
            return;
        }   

    }
    
    else{
        // If no RGVs have been detected, keep searching
        phase = "Search";
    }

    
}

void MissionPlanner::trail_phase(){

    // Make sure the correct cameras are activated
    set_cameras(true, true);

    //Set the lock on rgv A if no lock is set and rgv A is in view and not localized
    set_lock();

    if (out_of_bounds(drone.state)){
        ROS_INFO("Drone is out of bounds. Moving to boundary control phase...");
        phase = "Boundary Control";
        out_of_bounds_time = ros::Time::now();
    }

    // RGV A is detected
    else if (target_lock == "RGV A"){

        // Is RGV A stopped?
        if (rgvAStopped()) //rgvStopped returns a vector of two bools for [RGVA, RGVB]
        {
            // If RGV A is stopped, move to the coarse phase
            phase = "Coarse";
            coarse_time_engaged = ros::Time::now();
            ROS_INFO("RGV A has stopped. Moving to coarse phase...");
            thought = "Coarse";
            say(thought);
        }

        else{
            // If RGV A is not stopped, keep trailing it
            //phase = "Trail";
            ROS_INFO("RGV A has not stopped. Continuing trail phase...");
        }
    }

    // RGV B is detected
    else if(target_lock == "RGV B"){

        // Is RGV B stopped?
        if (rgvBStopped()) //rgvStopped returns a vector of two bools for [RGVA, RGVB]
        {
            // If RGV B is stopped, move to the coarse phase
            phase = "Coarse";
            coarse_time_engaged = ros::Time::now();
            ROS_INFO("RGV B has stopped. Moving to coarse phase...");
            thought = "Coarse";
            say(thought);
        }

        else{
            // If RGV B is not stopped, keep trailing it
            //phase = "Trail";
            ROS_INFO("RGV B has not stopped. Continuing trail phase...");
        }
    }

    else{
        // If no RGVs are detected, Search for them
        phase = "Search";
        search_point_time = ros::Time::now();
        ROS_INFO("No RGVs detected for trailing, returning to search...");
        thought = "Lost RGV. Returning to search";
        say(thought);
    }

}

void MissionPlanner::coarse_phase(){

    // Make sure the correct cameras are activated
    set_cameras(true, true);

    //Set the lock on rgv A if no lock is set and rgv A is in view and not localized
    set_lock();

    if (out_of_bounds(drone.state)){
        ROS_INFO("Drone is out of bounds. Moving to boundary control phase...");
        phase = "Boundary Control";
        out_of_bounds_time = ros::Time::now();
    }

    // If rgv A is detected and not coarsely localized yet
    else if (target_lock == "RGV A" && !env.rgvACoarseComplete){
        // if (rgvAStopped()){
        if (true){
            // If the drone has been in the coarse phase for more than X seconds, move to the fine phase
            // The time_coarsely_localized variable only updates after losing the rgv
            //ros::Time coarse_length = (ros::Time::now() - coarse_time_engaged) + time_coarsely_localized;
            ros::Duration coarse_length = (ros::Time::now() - coarse_time_engaged) + time_coarsely_localized;

            if (coarse_length > ros::Duration(drone.coarse_duration)){
                phase = "Fine";
                fine_time_engaged = ros::Time::now();
                time_coarsely_localized = ros::Duration(0);
                time_finely_localized = ros::Duration(0);
                env.rgvACoarseComplete = true;
                ROS_INFO("RGV A has been coarsely localized. Moving to fine phase...");
                thought = "Localized, moving to fine phase";
                say(thought);
            }

            else{
                // If the drone has not been in the coarse phase for more than 10 seconds, keep localizing
                //phase = "Coarse";

                // Output how long it has been localizing for
                ROS_INFO("RGV A has been localized for %f seconds...", coarse_length.toSec());
                std::ostringstream stream;
                stream << std::fixed << std::setprecision(1) << coarse_length.toSec();
                std::string coarse_length_str = stream.str();
                
                thought = "Coarse localized for " + coarse_length_str + " seconds";
                say(thought);
            }
        } else {
            // The rgv has begun moving again, got back to trail phase
            phase = "Trail";
            ROS_INFO("RGV A has started moving again. Returning to trail phase...");
            thought = "Trailing";
            say(thought);
        }
    }

    else if (target_lock == "RGV A" && env.rgvACoarseComplete){
        // The drone was trailed, but already coarsely localized. Check if it has been fine localized
        phase = "Fine";
        fine_time_engaged = ros::Time::now();
        ROS_INFO("RGV A has been coarsely localized already. Moving to fine phase...");
        thought = "Fine";
        say(thought);
    }

    // If rgv B is detected and not finely localized yet
    else if (target_lock == "RGV B"  && !env.rgvBCoarseComplete){
        // if (rgvBStopped()){
        if (true){
            // If the drone has been in the coarse phase for more than X seconds, move to the fine phase
            //ros::Time coarse_length = (ros::Time::now() - coarse_time_engaged) + time_coarsely_localized;
            ros::Duration coarse_length = (ros::Time::now() - coarse_time_engaged) + time_coarsely_localized;
            if (coarse_length > ros::Duration(drone.coarse_duration)){
                phase = "Fine";
                fine_time_engaged = ros::Time::now();
                time_coarsely_localized = ros::Duration(0);
                time_finely_localized = ros::Duration(0);
                env.rgvBCoarseComplete = true;
                ROS_INFO("RGV B has been coarsely localized. Moving to fine phase...");
                thought = "Localized, moving to fine phase";
                say(thought);
            }

            else{
                // If the drone has not been in the coarse phase for more than 10 seconds, keep localizing
                //phase = "Coarse";

                // Output how long it has been localizing for
                ROS_INFO("RGV B has been localized for %f seconds...", coarse_length.toSec());
                std::ostringstream stream;
                stream << std::fixed << std::setprecision(1) << coarse_length.toSec();
                std::string coarse_length_str = stream.str();
                
                thought = "Coarse localized for " + coarse_length_str + " seconds";
                say(thought);
            }
        } else {
            // The rgv has begun moving again, got back to trail phase
            phase = "Trail";
            ROS_INFO("RGV B has started moving again. Returning to trail phase...");
            thought = "Trailing";
            say(thought);
        }
    }

    else if (target_lock == "RGV B"  && env.rgvBCoarseComplete){
        // The drone was trailed, but already coarsely localized. Check if it has been fine localized
        phase = "Fine";
        fine_time_engaged = ros::Time::now();
        ROS_INFO("RGV B has been coarsely localized already. Moving to fine phase...");
    }

    else{
        // If no RGVs are detected, Search for them
        phase = "Search";
        search_point_time = ros::Time::now();
        ROS_INFO("No RGVs detected for localization, returning to search...");
        thought = "Lost RGV. Returning to search";
        say(thought);

        // Add the duration of the coarse phase to the search phase
        if (ros::Time::now() - last_rgvA_detection < ros::Time::now() - last_rgvB_detection){
            time_coarsely_localized = (last_rgvA_detection - coarse_time_engaged) + time_coarsely_localized;
        }
        else{
            time_coarsely_localized = (last_rgvB_detection - coarse_time_engaged) + time_coarsely_localized;
        }
        ROS_INFO("Time coarsely localized: %f", time_coarsely_localized.toSec());
    }
    return;
}

void MissionPlanner::fine_phase(){

    // Make sure the correct cameras are activated
    set_cameras(false, true);

    //Set the lock on rgv A if no lock is set and rgv A is in view and not localized
    set_lock();

    if (out_of_bounds(drone.state)){
        ROS_INFO("Drone is out of bounds. Moving to boundary control phase...");
        phase = "Boundary Control";
        out_of_bounds_time = ros::Time::now();
    }

    // If rgv A is detected and not finely localized yet
    else if (target_lock == "RGV A" && !env.rgvAFineComplete){
        // if (rgvAStopped()){
        if (true){
            // If the drone has been in the fine phase for more than X seconds, check if both have been localized
            ros::Duration fine_length = (ros::Time::now() - fine_time_engaged) + time_finely_localized;
            if (fine_length > ros::Duration(drone.fine_duration)){
                ROS_INFO("RGV A has been finely localized");
                thought = "Localized";
                say(thought);
                env.rgvAFineComplete = true;
                time_coarsely_localized = ros::Duration(0);
                time_finely_localized = ros::Duration(0);

                // If RGV B has been finely localized as well, move to the joint phase
                if (env.rgvBFineComplete){
                    phase = "Joint";
                    thought = "Joint";
                    say(thought);
                    joint_time_engaged = ros::Time::now();
                    joint_last_detection = ros::Time::now();
                    env.rgvAFineComplete = true;
                    ROS_INFO("Beginning joint phase...");
                }

                else{
                    // Search for RGV B
                    phase = "Search";
                    reset_lock();
                    ROS_INFO("RGV B has not been finely localized. Searching for RGV B...");
                }
            }

            else{
                // Output how long it has been localizing for
                ROS_INFO("RGV A has been finely localized for %f seconds...", fine_length.toSec());
                
                std::ostringstream stream;
                stream << std::fixed << std::setprecision(1) << fine_length.toSec();
                std::string fine_length_str = stream.str();
                
                thought = "Fine Localized for " + fine_length_str + " seconds";
                say(thought);
            }
        } else {
            // The rgv has begun moving again, got back to trail phase
            phase = "Trail";
            ROS_INFO("RGV A has started moving again. Returning to trail phase...");
            thought = "Trailing";
            say(thought);
        }
    }

    // If rgv B is detected and not finely localized yet
    else if (target_lock == "RGV B" && !env.rgvBFineComplete){
        // if (rgvBStopped()){
        if (true){
            // If the drone has been in the fine phase for more than X seconds, check if both have been localized
            ros::Duration fine_length = (ros::Time::now() - fine_time_engaged) + time_finely_localized;
            if (fine_length > ros::Duration(drone.fine_duration)){
                ROS_INFO("RGV B has been finely localized");
                thought = "Localized";
                say(thought);
                env.rgvBFineComplete = true;
                time_coarsely_localized = ros::Duration(0);
                time_finely_localized = ros::Duration(0);

                // If RGV A has been finely localized as well, move to the joint phase
                if (env.rgvAFineComplete){
                    phase = "Joint";
                    thought = "Joint";
                    say(thought);
                    joint_time_engaged = ros::Time::now();
                    joint_last_detection = ros::Time::now();
                    env.rgvBFineComplete = true;
                    ROS_INFO("Beginning joint phase...");
                }

                else{
                    // Search for RGV A
                    phase = "Search";
                    reset_lock();
                    ROS_INFO("RGV A has not been finely localized. Searching for RGV A...");
                }
            }

            else{
                // Output how long it has been localizing for
                ROS_INFO("RGV B has been finely localized for %f seconds...", fine_length.toSec());
                std::ostringstream stream;
                stream << std::fixed << std::setprecision(1) << fine_length.toSec();
                std::string fine_length_str = stream.str();
                
                thought = "Localized for " + fine_length_str + " seconds";
                say(thought);
            }
        } else {
            // The rgv has begun moving again, got back to trail phase
            phase = "Trail";
            ROS_INFO("RGV B has started moving again. Returning to trail phase...");
            thought = "Trailing";
            say(thought);
        }
    }

    else{
        // If no RGVs are detected, Search for them
        phase = "Search";
        search_point_time = ros::Time::now();
        ROS_INFO("No RGVs detected for localization, returning to search...");
        thought = "Lost RGV. Returning to search";
        say(thought);
                // Add the duration of the coarse phase to the search phase

        // Add the duration of the fine phase to the counter
        if (ros::Time::now() - last_rgvA_detection < ros::Time::now() - last_rgvB_detection){
            time_finely_localized = (last_rgvA_detection - fine_time_engaged) + time_finely_localized;
        }
        else{
            time_finely_localized = (last_rgvB_detection - fine_time_engaged) + time_finely_localized;
        }
    }
    return;
}

void MissionPlanner::joint_phase(){

    // Make sure the correct cameras are activated
    set_cameras(true, true);

    // Includes the duration of the current consistent joint phase + last time the rgvs were jointly localized
    ros::Duration joint_length = (ros::Time::now() - joint_time_engaged) + time_joint_localized;

    // If the drone is out of bounds, move to the boundary control phase
    if (out_of_bounds(drone.state)){
        ROS_INFO("Drone is out of bounds. Moving to boundary control phase...");
        phase = "Boundary Control";
        out_of_bounds_time = ros::Time::now();
    }

    else if (!env.rgvAInView || !env.rgvBInView){
        // If either rgvs are not in view, reset the joint phase timer
        joint_time_engaged = ros::Time::now();

        // If the drone lost one or both of the RGVs, return to the search phase
        if ((ros::Time::now() - joint_last_detection) > ros::Duration(drone.joint_detection_duration)){
            phase = "Joint Search";
            ROS_INFO("RGVs have been lost. Returning to search...");
            thought = "Lost RGVs. Joint search";
            say(thought);
        }
    }

    // If the drone has been in the joint phase for more than X seconds, return to home
    else if (joint_length > ros::Duration(drone.joint_duration)){
        env.jointComplete = true;
        phase = "Return Home";
        ROS_INFO("Joint phase has been completed. Returning to home...");
        thought = "I am free! Returning home!";
        say(thought);
    }

    else if (env.rgvAInView && env.rgvBInView){
        // Output how long it has been in the joint phase
        ROS_INFO("Joint phase has been engaged for %f seconds...", joint_length.toSec());
        
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(1) << joint_length.toSec();
        std::string joint_length_str = stream.str();
        
        thought = "Joint for " + joint_length_str + " seconds";
        

        say(thought);
        joint_last_detection = ros::Time::now();
    }

    // else{
    //     // If the drone lost one or both of the RGVs, return to the search phase
    //     if (!env.rgvAInView || !env.rgvBInView){
    //         phase = "Search";
    //         ROS_INFO("One or both RGVs have been lost. Returning to search...");
    //     }
    //         // Output how long it has been in the joint phase
    //         ROS_INFO("Joint phase has been engaged for %f seconds...", (ros::Time::now() - joint_time_engaged).toSec());
    // }
    return;
}

void MissionPlanner::joint_search_phase(){

    // Make sure the correct cameras are activated
    set_cameras(true, true);

    // If the drone is out of bounds, move to the boundary control phase
    if (out_of_bounds(drone.state)){
        ROS_INFO("Drone is out of bounds. Moving to boundary control phase...");
        phase = "Boundary Control";
        out_of_bounds_time = ros::Time::now();
    }

    // Stays in the search phase until both RGVs are found at least once
    if (env.rgvAInView && !env.JS_rgvA_detected){
        env.JS_rgvA_detected = true;
        ROS_INFO("RGV A has been detected. Continuing joint search...");
        thought = "RGV A detected";
        say(thought);
    }

    if (env.rgvBInView && !env.JS_rgvB_detected){
        env.JS_rgvB_detected = true;
        ROS_INFO("RGV B has been detected. Continuing joint search...");
        thought = "RGV B detected";
        say(thought);
    }

    if (env.JS_rgvA_detected && env.JS_rgvB_detected){
        phase = "Joint";
        joint_time_engaged = ros::Time::now();
        joint_last_detection = ros::Time::now();
        ROS_INFO("Both RGVs have been detected. Moving to joint phase...");
        thought = "Joint";
        say(thought);
    }
}

void MissionPlanner::set_lock(){
    // If the drone has not locked onto an RGV, lock onto the first one it sees
    if (target_lock == "None"){

        // Lock on to the first RGV it sees, but only if they still need to be localized
        if (env.rgvAInView && (!env.rgvACoarseComplete || !env.rgvAFineComplete)){
            target_lock = "RGV A";
            ROS_INFO("Locking onto RGV A...");
        }
        else if (env.rgvBInView && (!env.rgvBCoarseComplete || !env.rgvBFineComplete)){
            target_lock = "RGV B";
            ROS_INFO("Locking onto RGV B...");
        }
    } else {

        // Check if the drone has lost detection of a locked RGV
        if (target_lock == "RGV A" && ros::Time::now() - last_rgvA_detection > ros::Duration(drone.detection_duration)){
            target_lock = "None";
            ROS_INFO("Lost detection of RGV A. Releasing lock...");
        }
        else if (target_lock == "RGV B" && ros::Time::now() - last_rgvB_detection > ros::Duration(drone.detection_duration)){
            target_lock = "None";
            ROS_INFO("Lost detection of RGV B. Releasing lock...");
        }
    }
}

void MissionPlanner::reset_lock(){
    target_lock = "None";
    ROS_INFO("Releasing lock...");
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
    } else if (phase == "Joint Search") {
        return search_motion(waypoint);
    } else if (phase == "Return Home") {
        return return_home_motion(waypoint);
    } else if (phase == "ABORT") {
        ROS_INFO("Landing drone...");
        land();
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

    filter_waypoint(waypoint);
    return waypoint;
}

std::vector<double> MissionPlanner::return_home_motion(std::vector<double> waypoint) {
    /* Set waypoint to home location
     * Input:
     *         waypoint - 1x4 double vector of previously commanded point & yaw
     * Output:
     *         waypoint - 1x4 double vector of commanded point & yaw
     */
    waypoint[0] = drone.home[0];
    waypoint[1] = drone.home[1];
    waypoint[2] = env.bounds[0][2];
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
    if (check_waypoint_reached(drone.epsilon) == 1 || (ros::Time::now() - search_point_time) > ros::Duration(drone.search_point_duration)){

        // Output that the drone has reached the commanded point
        //std::cout << "Destination reached!" << std::endl;
        search_point_time = ros::Time::now();

        // Iterate the search location tracker in the environment object
        waypoint = env.next_searchpoint();
    }

    // Convert the waypoint to ENU coordinates
    waypoint = local_2_enu(waypoint);

    //waypoint[3] = getYaw(waypoint);

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

    // if (env.rgvAInView && env.rgvBInView) {
    //     // if both RGVs are in view, follow closest one
    //     if (isRGVAClosest()) {
    //         waypoint[0] = env.rgvAPosition[0];
    //         waypoint[1] = env.rgvAPosition[1];
    //     }
    //     else {
    //         waypoint[0] = env.rgvBPosition[0];
    //         waypoint[1] = env.rgvBPosition[1];
    //     }
    // }
    // else if (env.rgvAInView) {
    //     // if RGV-A is in view, follow it
    //     waypoint[0] = env.rgvAPosition[0];
    //     waypoint[1] = env.rgvAPosition[1];
    // }
    // else if (env.rgvBInView) {
    //     // if RGV-B is in view, follow it
    //     waypoint[0] = env.rgvBPosition[0];
    //     waypoint[1] = env.rgvBPosition[1];
    // }
    // else if (env.rgvAPosition[3] > env.rgvBPosition[3]) {
    //     // if no RGV is in view, but most recent rgv detected is RGV-A, target the last known position of it
    //     waypoint[0] = env.rgvAPosition[0];
    //     waypoint[1] = env.rgvAPosition[1];
    // }
    // else {
    //     // if no RGV is in view, but most recent rgv detected is RGV-B, target the last known position of it
    //     waypoint[0] = env.rgvBPosition[0];
    //     waypoint[1] = env.rgvBPosition[1];
    // }

    // New trail motion using target locking
    if (target_lock == "RGV A"){
        waypoint[0] = env.rgvAPosition[0];
        waypoint[1] = env.rgvAPosition[1];
    }
    else if (target_lock == "RGV B"){
        waypoint[0] = env.rgvBPosition[0];
        waypoint[1] = env.rgvBPosition[1];
    }

    // If the drone is not locked, the waypoint will not be changed

    waypoint[3] = getYaw(waypoint);
    waypoint[2] = drone.trail_altitude;

    waypoint = filter_waypoint(waypoint);

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

    waypoint = {x, y, drone.coarse_altitude, drone.theta*(180/M_PI)};

    waypoint = filter_waypoint(waypoint);

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

    //waypoint[3] = getYaw(waypoint);
    waypoint[2] = drone.fine_altitude;

    waypoint = filter_waypoint(waypoint);

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
    //std::vector<double> v1, v2, crossed;
    double midpoint[2] = {(rgvAPos[0]+rgvBPos[0])/2, (rgvAPos[1]+rgvBPos[1])/2};
    double width, desiredHeight, yaw;

    // if both RGVs are in view, optimize z height
    width = norm({rgvAPos[0] - rgvBPos[0], rgvAPos[1] - rgvBPos[1]}) / 2.0 + drone.fovWide*0.1;
    desiredHeight = width / tan(drone.fovWide*M_PI/360.0);
    if (desiredHeight > env.bounds[1][2]) {
        desiredHeight = env.bounds[1][2];
    }

    // Find the yaw angle as perpendicular to the line between the RGVs
     // Calculate the vector from rgvAPos to rgvBPos
    std::vector<double> v = {rgvBPos[0] - rgvAPos[0], rgvBPos[1] - rgvAPos[1]};

    // Calculate the yaw angle in radians
    yaw = atan2(v[1], v[0]);

    // Convert the yaw angle to degrees and make it perpendicular to the line between the RGVs
    yaw = yaw * (180.0 / M_PI);

    // Ensure the yaw angle is between 0 and 360
    if (yaw < 0) {
        yaw += 360;
    }

    waypoint = {(rgvAPos[0]+rgvBPos[0])/2, (rgvAPos[1]+rgvBPos[1])/2, desiredHeight, yaw};

    // if (env.rgvAInView && env.rgvBInView) {
    //     // if both RGVs are in view, optimize z height
    //     width = norm((rgvAPos.begin(), rgvAPos.end()-1) - (rgvBPos.begin(), rgvBPos.end()-1)) / 2.0 + drone.fovWide*0.1;
    //     desiredHeight = width / tan(drone.fovWide*M_PI/360.0);
    //     if (desiredHeight < env.bounds[1][2]) {
    //         desiredHeight = env.bounds[1][2];
    //     }

    //     if (rgvAPos[1] < rgvBPos[1]) {
    //         v1 = {rgvAPos[0]-rgvBPos[0], rgvAPos[1]-rgvBPos[1], 0};
    //     }
    //     else {
    //         v1 = {rgvBPos[0]-rgvAPos[0], rgvBPos[1]-rgvAPos[1], 0};
    //     }
    //     v2 = {1, 0, 0};
    //     yaw = (atan2(norm(cross(v1,v2)), dot(v1,v2)) - M_PI)*(180/M_PI);
    //     waypoint = {(rgvAPos[0]+rgvBPos[0])/2, (rgvAPos[1]+rgvBPos[1])/2, desiredHeight, yaw};
    // }
    // else {
    //     waypoint = {(rgvAPos[0]+rgvBPos[0])/2, (rgvAPos[1]+rgvBPos[1])/2, env.bounds[1][2], waypoint[3]};
    // }

    waypoint = filter_waypoint(waypoint);

    return waypoint;
}

// std::vector<double> MissionPlanner::coarse(std::vector<double> waypoint) {
//     /* Circles around an RGV
//      * Output:
//      *         waypoint - 1x3 double vector of commanded point
//      * Makes use of the uas theta iterative property to track where in the orbit
//      * the drone is. Makes additional adjustments to the commanded point to
//      * avoid exiting the boundary, when necessary. 
//      */
//     std::vector<double> target;
//     double x, y;
//     double thetaStep = 6 * M_PI/180; // change in circle angle over time step [rad]
//     int r = 10; // radius of the orbital path [m]

//     ////// TODO: add branches determining which RGV to target //////
//     // if both RGVs in view, target closest one
//     // if only one RGV in view, target that one
//     // if no RGV in view, target most recent RGV estimate (moving has put RGV out of camera view)
//     target = waypoint;

//     // now get the commanded x,y

//     if (drone.theta == -1) {
//         // if coarse hasn't started yet, initialize the starting angle based
//         // on current drone position
//         drone.theta = atan2(drone.state[1]-target[1], drone.state[0]-target[0]);
//     }
//     // increment the angle of the orbit by thetaStep
//     drone.theta += thetaStep;
//     // set the x and y of the commanded point
//     x = target[0] + r*cos(drone.theta);
//     y = target[1] + r*sin(drone.theta);

//     // if the commanded point would place the drone outside the bounds,
//     // change the commanded point to move along the boundary instead
//     if (x > (env.bounds[1][0]-1)) {
//         x = env.bounds[1][0]-1;
//     }
//     else if (x < (env.bounds[0][0]+1)) {
//         x = env.bounds[0][0]+1;
//     }
//     if (y > (env.bounds[1][1]-1)) {
//         y = env.bounds[1][1]-1;
//     }
//     else if (y < (env.bounds[0][1]+1)) {
//         y = env.bounds[0][1]+1;
//     }

//     waypoint = {x, y, env.bounds[0][2]};
//     return waypoint;
// }

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




///////////// Helper Functions //////////////////////
double MissionPlanner::getYaw(std::vector<double> waypoint) {
    /* Get the yaw angle to turn the drone towards the given waypoint
     * Input:
     *         waypoint - 1x4 double vector of commanded point & yaw
     * Output:
     *         yaw - double, yaw angle facing commanded point
     */

    //check if there is a big enough distance from the state to the waypoint
    if (pow(pow(drone.state[0]-env.rgvAPosition[0], 2) + pow(drone.state[1]-env.rgvAPosition[1], 2), 0.5) < 0.5){
        return drone.state[3];
    }
    return atan2(waypoint[0] - drone.state[0], waypoint[1] - drone.state[1])*(180/M_PI)-90;
}



void MissionPlanner::output_drone_state(){
    std::vector<double> dronePos = drone.state;
    // Output the current location of the UAS and the next location it is going to
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Current Position: " << dronePos[0] << ", " << dronePos[1] << ", " << dronePos[2] << " -> Going to " << drone.dest[0] << ", " << drone.dest[1] << ", " << drone.dest[2] << std::endl;
    //enu_2_local(dronePos);

}



bool MissionPlanner::out_of_bounds(std::vector<double> waypoint){
    // // Check if a waypoint would fall outside of the environment bounds
    // std::vector<std::vector<double>> bounds_g = {local_2_enu(env.bounds[0]), local_2_enu(env.bounds[1])};

    // if (waypoint[0] < bounds_g[0][0] || waypoint[0] > bounds_g[1][0] || waypoint[1] < bounds_g[0][1] || waypoint[1] > bounds_g[1][1]){
    //     output_drone_state();
    //     ROS_ERROR("Bounds are x: %f to %f, y: %f to %f. Waypoint is out of bounds!", bounds_g[0][0], bounds_g[1][0], bounds_g[0][1], bounds_g[1][1]);
    //     return true;
    // }

    //return false;


    // if (waypoint[0] < env.bounds[0][0] || waypoint[0] > env.bounds[1][0] || waypoint[1] < env.bounds[0][1] || waypoint[1] > env.bounds[1][1] || waypoint[2] < env.bounds[0][2]){
    //     output_drone_state();
    //     ROS_ERROR("Bounds are x: %f to %f, y: %f to %f. Waypoint is out of bounds!", + env.bounds[0][0], env.bounds[1][0], env.bounds[0][1], env.bounds[1][1]);
    //     say("Out of bounds");
    //     return true;
    // }


    if (waypoint[2] < env.bounds[0][2] - 10.0){
        output_drone_state();
        ROS_ERROR("Bounds are x: %f to %f, y: %f to %f. Waypoint is out of bounds!", + env.bounds[0][0], env.bounds[1][0], env.bounds[0][1], env.bounds[1][1]);
        say("Out of bounds");
        return true;
    }

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

// Returns a vector of a bool for if both RGVs are stopped
bool MissionPlanner::rgvAStopped(){
    /* Returns a vector of a bool for if both RGVs are stopped
     * Input:  
     *         env   - enviornment object
     * Output: 
     *         rgvStopped - vector of bools for [RGVA, RGVB]
     */

    bool rgvStopped = true;

    // check if the RGVs are stopped by getting the deviation of their positions
    double std_x = standard_deviation(env.rgvAHistory.x_pos);
    double std_y = standard_deviation(env.rgvAHistory.y_pos);
    double std = sqrt(pow(std_x, 2) + pow(std_y, 2));

    // Compare the standard deviation of the positions form the time history to the threshold set manually
    if (std > drone.stopped_threshold){
        rgvStopped = false;
    }


    return rgvStopped;
}

bool MissionPlanner::rgvBStopped(){
    /* Returns a vector of a bool for if both RGVs are stopped
     * Input:  
     *         env   - enviornment object
     * Output: 
     *         rgvStopped - vector of bools for [RGVA, RGVB]
     */

    bool rgvStopped = true;

    // check if the RGVs are stopped by getting the deviation of their positions
    double std_x = standard_deviation(env.rgvBHistory.x_pos);
    double std_y = standard_deviation(env.rgvBHistory.y_pos);
    double std = sqrt(pow(std_x, 2) + pow(std_y, 2));

    // Compare the standard deviation of the positions form the time history to the threshold set manually
    if (std > drone.stopped_threshold){
        rgvStopped = false;
    }

    return rgvStopped;
}

void MissionPlanner::set_primary_activated(bool activated){
    primary_activated = activated;
}

void MissionPlanner::set_secondary_activated(bool activated){
    secondary_activated = activated;
}

bool MissionPlanner::get_primary_activated(){
    return primary_activated;
}

bool MissionPlanner::get_secondary_activated(){
    return secondary_activated;
}

void MissionPlanner::set_cameras(bool primary, bool secondary){
    if (primary_activated != primary){
        primary_activated = primary;
    }

    if (secondary_activated != secondary){
        secondary_activated = secondary;
    }
}

void MissionPlanner::say(std::string message){
    std_msgs::String speak_msg;
    speak_msg.data = message;
    speak_pub.publish(speak_msg);
}

std::vector<double> MissionPlanner::filter_waypoint(std::vector<double> waypoint){
    // Check if the waypoint is too far from the drone's position
    double dist = pow(pow(drone.state[0]-waypoint[0], 2) + pow(drone.state[1]-waypoint[1], 2), 0.5);
    if (dist > drone.max_range){
        // get the normal vector to the waypoint from the drone
        std::vector<double> normal = {(waypoint[0]-drone.state[0])/dist, (waypoint[1]-drone.state[1])/dist};
        // scale the normal vector to the max range
        waypoint[0] = drone.state[0] + normal[0]*drone.max_range;
        waypoint[1] = drone.state[1] + normal[1]*drone.max_range;
    }

    return waypoint;
}
