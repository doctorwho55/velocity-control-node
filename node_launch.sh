#!/bin/bash

# setup ros environment
source "/node-ws/devel/setup.bash"
# roslaunch joy_cli joy_cli.launch veh:=$DUCKIEBOT_NAME
# roslaunch vel_func_node vel_func_node.launch veh:=$DUCKIEBOT_NAME
roslaunch vehicle_detection vehicle_follower.launch veh:=$DUCKIEBOT_NAME
