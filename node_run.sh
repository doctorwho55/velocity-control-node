#!/bin/bash

# docker -H $1.local run -it --network=host -e ROS_MASTER_URI='http://10.42.0.19:11311' -e DUCKIEBOT_NAME='duckiebot02' -e VEHICLE_NAME='duckiebot02' doctorwho55/velocity-control-node:master
docker -H $1.local run -it --network=host -e ROS_MASTER_URI='http://10.42.0.19:11311' -e DUCKIEBOT_NAME=$1 -e VEHICLE_NAME=$1 doctorwho55/velocity-control-node:master

