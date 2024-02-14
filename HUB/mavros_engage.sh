#!/bin/bash

xterm -T "mavros" -e "roslaunch mavros apm.launch fcu_url:=/dev/ttyTHS1:57600@" &
