#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include<algorithm>
#include<iterator>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


using namespace std;
using namespace Eigen;

class GridMap{
    private:
    public:
};

float robot_pose[3];
vector<float> laser_reads;
float map_resolution = 0.05;
float map_origin_x = 500.00;
float map_origin_y = 500.00;
int map_height = 1000;
int map_width = 1000;
float map_grid[1000][1000];
float angle_increase = 0.00436;

nav_msgs::OccupancyGrid create_map(){
    cout << "CREATING MAP" << endl;
    nav_msgs::OccupancyGrid map = nav_msgs::OccupancyGrid();
    map.header.frame_id = "/map_out";
    map.header.seq = 1;
    map.info.resolution = 0.05;
    geometry_msgs::Pose origin;
    origin.position.x = -500;
    origin.position.y = -500;
    origin.position.z = 0;
    map.info.origin = origin;
    map.info.height = 1000;
    map.info.width = 1000;
    std::vector<int8_t> m(map_height*map_width,-1);  
    cout << "REACH UNTIL HERE" << endl;
    copy(m.begin(), m.end(), back_inserter(map.data));
    cout << "ALL MAP IS UNKNOW" << endl;

    return map;
}

void createMatrix(){
    float map_grid[map_height][map_width];

}

std::tuple<int,int> odom2cell(float odom_pose_x, float odom_pose_y){
    int i = odom_pose_x/map_resolution - map_origin_x/map_resolution;
    int j = odom_pose_y/map_resolution - map_origin_y/map_resolution;
    return std::make_tuple(i,j);
}

std::tuple<float,float> cell2odom(int cell_value_x, int cell_value_y){
    float x = (cell_value_x + map_origin_x/map_resolution) * map_resolution;
    float y = (cell_value_y + map_origin_y/map_resolution) * map_resolution;
    return std::make_tuple(x,y);
}

void laserscan_callback(const sensor_msgs::LaserScan::ConstPtr& laserscan_msg){
    for(int x = 0; x < laserscan_msg->ranges.size(); x++){
        laser_reads.emplace_back(laserscan_msg->ranges[x]);
        // cout << "LASER_READS: " << laser_reads[x] << " HUSKY_SCAN: " << laserscan_msg->ranges[x] << endl;
        // if(laserscan_msg->ranges[x] == INFINITY){
        //     cout << "TA FAZENDO O CALLBACK." << endl;
        // }
    }
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg){
    robot_pose[0] = odom_msg->pose.pose.position.x;
    robot_pose[1] = odom_msg->pose.pose.position.y;
    Quaternionf q;
    q.x() = odom_msg->pose.pose.orientation.x;
    q.y() = odom_msg->pose.pose.orientation.y;
    q.z() = odom_msg->pose.pose.orientation.z;
    q.w() = odom_msg->pose.pose.orientation.w;
    auto euler = q.toRotationMatrix().eulerAngles(0,1,2);
    robot_pose[2] = euler[2];
    cout << "YAW: " << euler[2] << endl;
}

void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_msg){
    robot_pose[0] = amcl_msg->pose.pose.position.x;
    robot_pose[1] = amcl_msg->pose.pose.position.y;
    Quaternionf q;
    q.x() = amcl_msg->pose.pose.orientation.x;
    q.y() = amcl_msg->pose.pose.orientation.y;
    q.z() = amcl_msg->pose.pose.orientation.z;
    q.w() = amcl_msg->pose.pose.orientation.w;
    auto euler = q.toRotationMatrix().eulerAngles(0,1,2);
    robot_pose[2] = euler[2];
    cout << "YAW: " << euler[2] << endl;
}

std::tuple<float,float> getLaserPosition(int index){
    if (laser_reads[index] == INFINITY) return std::make_tuple(0,0);

    float laser_x = (cos((index*angle_increase)+robot_pose[2])*laser_reads[index]);
    float laser_y = (sin((index*angle_increase)+robot_pose[2])*laser_reads[index]);
    return std::make_tuple(laser_x,laser_y);
}

nav_msgs::OccupancyGrid update_map(nav_msgs::OccupancyGrid map){
    cout << "ENTERING MAP UPDATE" << endl;
    int cell_x, cell_y, laser_cell_x, laser_cell_y;
    float odom_laser_x, odom_laser_y;
    std::tie(cell_x, cell_y) = odom2cell(robot_pose[0],robot_pose[1]);
    cout << "CELL_X_ROBOT: " << cell_x << " CELL_Y_ROBOT: " << cell_y << endl;
    if (cell_x >= 0 && cell_x < map_width && cell_y >= 0 && cell_y < map_height) {
        map_grid[cell_x][cell_y] = 100;
    }
    
    // for (int i = 0; i < laser_reads.size(); i++) {
    //     std::tie(odom_laser_x,odom_laser_y) = getLaserPosition(i);
    //     cout << "ODOM_LASER_X: " << odom_laser_x << " ODOM_LASER_Y: " << odom_laser_y << endl;
    //     std::tie(laser_cell_x,laser_cell_y) = odom2cell(odom_laser_x+robot_pose[0],odom_laser_y+robot_pose[1]);
    //     cout << "LASER_CELL_X: " << laser_cell_x << " LASER_CELL_Y: " << laser_cell_y << endl;
    //     if ((laser_cell_x >= 0 && laser_cell_y >= 0) && (laser_cell_x <= map_width && laser_cell_y <= map_height)) {
    //         map_grid[laser_cell_x][laser_cell_y] = 100;
    //     }
    // }
    
    for(int x = 0; x < map_width; x++){
        int multi = x * map_width;
        for(int y = 0; y < map_height; y++){
            int index = y + multi;
            map.data[index] = (int8_t)map_grid[x][y];
        }
    }

    return map;
}

int main(int argc, char **argv){
    
    cout << "STARTING THIS PROGRAM!" << endl;

    for (int x = 0; x < map_width; x++) {
        for (int y = 0; y < map_height; y++) {
            map_grid[x][y] = -1;
        }
    }

    ros::init(argc, argv, "occupancy_grid");
    ros::NodeHandle node;

    nav_msgs::OccupancyGrid map = create_map();  

    ros::Subscriber laser_sub = node.subscribe<sensor_msgs::LaserScan>("/husky1/scan",10,laserscan_callback);
    ros::Subscriber odom_sub = node.subscribe("/husky1/amcl_pose",1000,amcl_callback);

    ros::Publisher map_pub = node.advertise<nav_msgs::OccupancyGrid>("/map_out",10);
    
    // map_pub.publish(map);
    ros::Rate rate(10);
    while(ros::ok()){
        map = update_map(map);
        map_pub.publish(map);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
