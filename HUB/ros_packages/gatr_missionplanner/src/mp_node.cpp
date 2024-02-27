#include "headers/mp_node_headers.h"

//include API 

int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");
	
	//initialize control publisher/subscribers
	init_publisher_subscriber(gnc_node);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(3);

	//specify some waypoints 
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	// nextWayPoint.x = 0;
	// nextWayPoint.y = 0;
	// nextWayPoint.z = 3;
	// nextWayPoint.psi = 0;
	// waypointList.push_back(nextWayPoint);


	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	int counter = 0;

	//initialize the uas and environment objects
	MissionPlanner mp;
	std::vector<float> curr_waypoint_new(4);
	std::vector<float> curr_waypoint_prev(4);

	// MAIN LOOP //
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();

		//Update the drone's position
		mp.update_drone_state();
		mp.output_drone_state();

		// Save the previous waypoint
		curr_waypoint_prev = curr_waypoint_new;

		////////// Original Square Code //////////
		// if(check_waypoint_reached(.3) == 1)
		// {
		// 	if (counter < waypointList.size())
		// 	{
		// 		set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
		// 		counter++;	
		// 	}else{
		// 		//land after all waypoints are reached
		// 		land();
		// 	}	
		// }	
		//////////////////////////////////////////

		//////////// Only Search Code ////////////
		curr_waypoint_new = mp.bounds_trace(curr_waypoint_prev);

		//////////////////////////////////////////

		// If the waypoint has changed, set the new waypoint
		if (curr_waypoint_new != curr_waypoint_prev) {

			// Set the new waypoint
			float heading_angle = 0; //Nolan can you make this the same as the LTL planner?
			set_destination(curr_waypoint_new[0], curr_waypoint_new[1], curr_waypoint_new[2], heading_angle);
		}

	}
	return 0;
}