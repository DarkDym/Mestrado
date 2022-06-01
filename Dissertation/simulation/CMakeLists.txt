cmake_minimum_required(VERSION 3.1)
project(dissertation_simulation)

find_package(catkin REQUIRED
    gazebo_ros
    gazebo_ros_control
    controller_manager
    velocity_controllers
    roscpp
    rospy
    apriltag_ros
    el
)

catkin_package(
	CATKIN_DEPENDS 
        roscpp 
        rospy 
        apriltag_ros
        el 
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

add_executable(tag_listener scripts/path_fromtag.cpp)
target_link_libraries(tag_listener ${catkin_LIBRARIES} )


# install roslaunch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
