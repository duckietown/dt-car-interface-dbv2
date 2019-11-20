#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
roslaunch --wait car_interface_dbv2 all.launch veh:=$VEHICLE_NAME
