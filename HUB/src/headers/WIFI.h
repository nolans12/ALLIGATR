#pragma once

#include "CORE.h"

// $$$$$$$$$$$$$$ DESCRIPTION $$$$$$$$$$$$$$$$
// This class will contain all the functions needed for the HUB to run.

class WIFI{
    public:
        // $$$$$$$$$$$$$$ CONSTRUCTOR $$$$$$$$$$$$$$$$
        WIFI();

        // $$$$$$$$$$$$$$ DESTRUCTOR $$$$$$$$$$$$$$$$
        //~WIFI();

        // $$$$$$$$$$$$$$ FUNCTIONS $$$$$$$$$$$$$$$$

        /// @brief Pings all devices on the connected network.
        /// @return n of successful pings, -1 if not connected to a network.
        int ping_all();

};