# Reset the sensor
echo 1 > /sys/class/gpio/gpio145/value
sleep 5
echo 0 > /sys/class/gpio/gpio145/value
sleep 1

# Start launch file
#roslaunch se3control nanoplus_sim.launch
#roslaunch se3control combo.launch
roslaunch se3control nanoplus.launch
