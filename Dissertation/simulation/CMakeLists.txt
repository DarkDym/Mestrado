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
    mmm
    el
    darknet_ros
    darknet_ros_msgs
)

# set(CMAKE_PREFIX_PATH "/home/phi/m_explore_ws/devel/")
# find_package(mmm REQUIRED)
# find_package(el REQUIRED)

catkin_package(
	CATKIN_DEPENDS 
        roscpp 
        rospy 
        apriltag_ros
        mmm
        el
        darknet_ros
        darknet_ros_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(tag_listener scripts/path_fromtag.cpp)
target_link_libraries(tag_listener ${catkin_LIBRARIES} )

add_executable(map_server scripts/map_server_teste.cpp)
target_link_libraries(map_server ${catkin_LIBRARIES})

add_executable(client_move scripts/client_move.cpp)
target_link_libraries(client_move ${catkin_LIBRARIES})

add_executable(grid_map scripts/grid_map.cpp)
target_link_libraries(grid_map ${catkin_LIBRARIES})

add_executable(patrol_move scripts/patrol.cpp)
target_link_libraries(patrol_move ${catkin_LIBRARIES})

add_executable(semantic scripts/semantic.cpp)
target_link_libraries(semantic ${catkin_LIBRARIES})

# install roslaunch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
