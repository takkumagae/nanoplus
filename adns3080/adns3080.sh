#!/bin/bash

# Reset the sensor
echo 1 > /sys/class/gpio/gpio145/value
sleep 5
echo 0 > /sys/class/gpio/gpio145/value
sleep 1

# Start launch file
roslaunch adns3080 test.launch
#roslaunch px4flow_node px4flow.launch
