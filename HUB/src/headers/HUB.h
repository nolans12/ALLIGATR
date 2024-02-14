#pragma once

#include "CORE.h"

// $$$$$$$$$$$$$$ DESCRIPTION $$$$$$$$$$$$$$$$
// This class will contain all the functions needed for the HUB to run.

class HUB{
    public:
        // $$$$$$$$$$$$$$ CONSTRUCTOR $$$$$$$$$$$$$$$$
        HUB();

        // $$$$$$$$$$$$$$ DESTRUCTOR $$$$$$$$$$$$$$$$
        ~HUB();

        // $$$$$$$$$$$$$$ FUNCTIONS $$$$$$$$$$$$$$$$

        /// @brief Sets the mode of the drone. Reads from the /mode topic.
        void set_mode();

        /// @brief Sets the mode of the drone to standby. Sets the /mode topic to "standby".
        //void set_standby();

        /// @brief Sets the mode of the drone to manual. Sets the /mode topic to "manual".
        //void set_manual();

        /// @brief Sets the mode of the drone to auto. Sets the /mode topic to "auto".
        //void set_auto();

};