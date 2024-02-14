// Main HUB Script

#include "src/headers/CORE.h"

/*
TO-DO and STRUCTURE:
    CAMERA (CLASS)
        -Object that can adjust focus, record, turn off and on, and take pictures, etc...

    WIFI (CLASS)
        -A check for wifi connection and return speed?
        -Does a send/recieve function need to be written or can we assume that ROS will handle this?

    MANUAL CONTROLLER
        -The controller plugs directly into the cube. We need a way to switch priority from the nano to the manual controller. 
        If this doesn't work, we can try to send manual controlls to the nano directly.

    CUBE (CLASS)
        - Establish connection to the cube, send waypoints, and recieve data from the cube.
*/

int main(){

    // Print the engaging message
    std::cout << "ENGAGING AUTO AL..." << std::endl;

    // $$$$$$$$$$$$$ PRECHECK LIST $$$$$$$$$$$$$$$$
        //WIFI CONNECTION

        //ROS CONNECTION

        //MANUAL CONTROLLER CONNECTION

        //CUBE CONNECTION

        //CAMERA CONNECTION

        //Wait until mode set to auto

        // Loop until user enters "exit"
        std::cout << "Type 'q' to quit the HUB and close all processes." << std::endl;
        while(true){
            // Check for user input
            std::string input;
            std::cin >> input;

            // If user enters "exit", break the loop
            if(input == "q"){
                break;
            }
        }

    return 0;
}
