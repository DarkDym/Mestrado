cmake_minimum_required(VERSION 3.1)
project(dissertation_simulation)

set(CMAKE_MODULE_PATH /home/phi/m_explore_ws/src/m-explore/map_merge)

find_package(catkin REQUIRED
    gazebo_ros
    gazebo_ros_control
    controller_manager
    velocity_controllers
    roscpp
    rospy
    apriltag_ros
)

find_package(mmm COMPONENTS)

catkin_package(
	CATKIN_DEPENDS 
        roscpp 
        rospy 
        apriltag_ros
        mmm
        el
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

include_directories(
  /home/phi/m_explore_ws/devel
)

add_executable(tag_listener scripts/path_fromtag.cpp)
target_link_libraries(tag_listener ${catkin_LIBRARIES} )


# install roslaunch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
