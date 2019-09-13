#!/bin/bash

sudo killall rosmaster
sudo killall gzserver
sudo killall gzclient
roslaunch dut_gazebo dut_world.launch
