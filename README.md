# delevery-robot

INSTRUCTION:

1. INSTALL ROS NOETIC(UBUNTU 20.04)
2. INSTALL LIBARIES:
	A) SEARCH IN LIB MANAGER IN ARDUINO IDE - DigiPotX9Cxxx 
	B) ROSSERIAL ARDUINO LIBARY (only 0.7.9 v)
3. INSTALL ROSBRIDGE - https://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge
4.Start the ROS Master - roscore
5.Run rosserial client on the machine - rosrun rosserial_python serial_node.py /dev/tty<USB# or ACM#>
5.Run rosbridge - roslaunch rosbridge_server rosbridge_websocket.launch
6. open page.html and press wasd for moving robot)