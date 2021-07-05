# roslaunch mavros px4.launch fcu_url:="udp://:20101@192.168.1.176:20100" & PID1=$!
# sleep 10s
# roslaunch rflysim_ros_pkg camera.launch & PID2=$!
# sleep 10s
roslaunch obs rflysim_sphere.launch & PID1=$!
sleep 2s
roslaunch obs obs.launch & PID2=$!

# exit
wait
kill PID1 PID2
exit
