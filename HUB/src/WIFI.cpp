
#include "headers/WIFI.h"
#include <sstream>
#include <vector>
#include <iterator>

// $$$$$$$$$$$$$$ CONSTRUCTOR $$$$$$$$$$$$$$$$
WIFI::WIFI(){
    // $$$$$$$$$$$$$$ CONSTRUCTOR $$$$$$$$$$$$$$$$
    // This function will be called when the WIFI object is created.
    // This will be used to initialize the WIFI Object.
}

/// @brief Pings all devices on the connected network.
/// @return n of successful pings, -1 if not connected to a network.
int WIFI::ping_all(){
    // $$$$$$$$$$$$$$ PING ALL $$$$$$$$$$$$$$$$
    // This function will ping all devices on the connected network.
    // If the ping is successful, the function will return the number of successful pings.
    // If the ping is unsuccessful, the function will return -1.
    std::cout << "WIFI ping is WIP - Returning 0" <<std::endl;

    return 0;
}