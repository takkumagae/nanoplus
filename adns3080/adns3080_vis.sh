#!/bin/bash
echo 1 > /sys/class/gpio/gpio145/value
sleep 1
echo 0 > /sys/class/gpio/gpio145/value
sleep 1
roslaunch adns3080 vis.launch
