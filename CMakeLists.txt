cmake_minimum_required(VERSION 2.8.3)
project(labview_bridge)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	rospy 
	telerob_msgs
)

catkin_python_setup()
