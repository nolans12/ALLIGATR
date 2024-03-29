#include "headers/mp_node_headers.h"

// // Global variable to hold the RGV detection status
// bool rgvAInView;

// // Callback function to be updated when the RGV is detected
// void rgvA_detected_callback(const std_msgs::Float32MultiArray::ConstPtr& coords)
// {
// 	// Display coords
// 	ROS_INFO("RGV Coords: [%f, %f]", coords->data[0], coords->data[1]);

// 	// Check if the RGV is in view
//     if(coords->data[0]*10000 !=0.0 || coords->data[1]*10000 != 0.0){
// 		//ROS_INFO("Coarsly localizing...");
// 		rgvAInView = true;
// 	}
// 	else{
// 		rgvAInView = false;
// 	}
    
// }

int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node;
	std::string pattern_name;

	// If the pattern_name string is not provided, default to full mission
	if (argc < 2) {
		pattern_name = "full mission";

		//Output the pattern name
		ROS_INFO("Engaging Full Mission Mode");
	}
	else {
		pattern_name = argv[1];
	
		//Output the pattern name
		ROS_INFO("User Specified Mode: %s", pattern_name.c_str());
	}
	
	//initialize control publisher/subscribers
	init_publisher_subscriber(gnc_node);

	// Create subscriber to rel_coord topic
	//ros::Subscriber rel_coord_sub = gnc_node.subscribe("CV/rel_coord_A", 10, rgvA_detected_callback);

	//initialize the uas and environment objects
	MissionPlanner mp(gnc_node);
	//rgvAInView = false;
	std::vector<double> curr_waypoint_new(4);
	std::vector<double> curr_waypoint_prev(4);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(3);

	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	int counter = 0;

	// MAIN LOOP //
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();

		//Update the drone's position
		mp.update_drone_state(curr_waypoint_new);
		//mp.output_drone_state();

		// Save the previous waypoint
		curr_waypoint_prev = curr_waypoint_new;
		
		// Check the command line argument to see what loop to run
		if (pattern_name == "bounds"){
			curr_waypoint_new = mp.bounds_trace(curr_waypoint_new);
		}
		else if (pattern_name == "search")
		{
			curr_waypoint_new = mp.search_motion(curr_waypoint_new);
		}
		else if (pattern_name == "locate")
		{
			// if (rgvAInView == true){
			// 	curr_waypoint_new = mp.coarse_motion(curr_waypoint_new);
			// }
			// else{
			curr_waypoint_new = mp.direct_locate(curr_waypoint_new);
			// }
		}
		else if (pattern_name == "full mission")
		{
			//////////// MAIN LOOP HERE ////////////
			
			// Determine the phase of the mission at the given timestep
			mp.determine_phase();

			// Execute the phase
			curr_waypoint_new = mp.determine_motion(curr_waypoint_new);

			// IF the phase is in ABORT, land the drone and exit the program
			if (mp.getPhase() == "ABORT") {
				land();
				while(ros::ok()){
					ros::spinOnce();
					rate.sleep();
				};
				return 0;
			}
			
			//////////////////////////////////////////
		}
		else{
			ROS_WARN("Mission pattern not recognized! Aborting mission...");
			continue;
		}

		

		// If the waypoint has changed, set the new waypoint
		if (curr_waypoint_new != curr_waypoint_prev) {

			//Check if the waypoint is out of bounds from the environment
			if (mp.out_of_bounds(curr_waypoint_new) == true) {
				ROS_WARN("Waypoint out of bounds! Aborting mission...");
				//land();
				//return 0;
			}

			// Set the new waypoint
			set_destination(curr_waypoint_new[0], curr_waypoint_new[1], curr_waypoint_new[2], curr_waypoint_new[3]);
		}

	}
	return 0;
}