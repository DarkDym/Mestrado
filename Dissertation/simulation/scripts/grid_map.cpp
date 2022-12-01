#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <math.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_listener.h>

using namespace std;
using namespace Eigen;

class GridMap{
    private:
    public:
};

float robot_pose[3];
float laser_pose[2];
vector<float> laser_reads;
sensor_msgs::LaserScan laser_reads_;
float map_resolution = 0.05;
float map_origin_x = -100.00;
float map_origin_y = -100.00;
int map_height = 4000;
int map_width = 4000;
int map_grid[4000][4000];
float angle_increase = 0.00436;
nav_msgs::OccupancyGrid map_out_;
vector<vector<int>> tst;

const float MAX_RANGE = 30.0;

nav_msgs::OccupancyGrid create_map(){
    cout << "CREATING MAP" << endl;
    nav_msgs::OccupancyGrid map = nav_msgs::OccupancyGrid();
    map.header.frame_id = "/map_out";
    map.header.seq = 1;
    map.info.resolution = 0.05;
    geometry_msgs::Pose origin;
    origin.position.x = -100.00;
    origin.position.y = -100.00;
    origin.position.z = 0;
    map.info.origin = origin;
    map.info.height = 4000;
    map.info.width = 4000;
    std::vector<int8_t> m(map_height*map_width,-1);
    copy(m.begin(), m.end(), back_inserter(map.data));
    cout << "ALL MAP IS UNKNOW" << endl;

    return map;
}

void create_vector_map(int height, int width){
    cout << "CREATING VECTOR MAP!" << endl;
    for (int vy = 0; vy < width; vy++) {
        vector<int> v_aux;
        for (int vx = 0; vx < height; vx++) {
            v_aux.push_back(-1);
        }
        tst.push_back(v_aux);
    }
    cout << "VECTOR MAP CREATED: " << tst[0][0] << " | SIZE OF VECTOR: " << tst.size() << endl;    
}

void grid_map_callback(const nav_msgs::OccupancyGridConstPtr& map_msg){
    map_out_.header.frame_id = map_msg->header.frame_id;
    map_out_.header.seq = map_msg->header.seq;
    map_out_.header.stamp = map_msg->header.stamp;
    map_out_.info.resolution = map_msg->info.resolution;
    map_out_.info.origin = map_msg->info.origin;
    map_out_.info.height = map_msg->info.height;
    map_out_.info.width = map_msg->info.width;
    map_out_.data = map_msg->data;
}

std::tuple<int,int> odom2cell(float odom_pose_x, float odom_pose_y){
    // int i = odom_pose_x/map_resolution - map_origin_x/map_resolution;
    // int j = odom_pose_y/map_resolution - map_origin_y/map_resolution;

    int i = odom_pose_x/map_out_.info.resolution - map_out_.info.origin.position.x/map_out_.info.resolution;
    int j = odom_pose_y/map_out_.info.resolution - map_out_.info.origin.position.y/map_out_.info.resolution;
    return std::make_tuple(j,i);
}

std::tuple<float,float> cell2odom(int cell_value_x, int cell_value_y){
    // float x = (cell_value_x + map_origin_x/map_resolution) * map_resolution;
    // float y = (cell_value_y + map_origin_y/map_resolution) * map_resolution;

    float x = (cell_value_x + map_out_.info.origin.position.x/map_out_.info.resolution) * map_out_.info.resolution;
    float y = (cell_value_y + map_out_.info.origin.position.y/map_out_.info.resolution) * map_out_.info.resolution;
    return std::make_tuple(x,y);
}

void laserscan_callback(const sensor_msgs::LaserScan::ConstPtr& laserscan_msg){
    laser_reads_ = *laserscan_msg;
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
}

std::tuple<float,float> getLaserPosition(int index){
    float robot_rad, laser_x, laser_y;
    if (laser_reads_.ranges[index] == INFINITY){
        robot_rad = remainder(robot_pose[2]+(((720-index)*angle_increase)-M_PI/2),2.0*M_PI);
        laser_x = (cos(robot_rad)*MAX_RANGE);
        laser_y = (sin(robot_rad)*MAX_RANGE);

        return std::make_tuple(laser_x,laser_y);
    }

    robot_rad = remainder(robot_pose[2]+(((720-index)*angle_increase)-M_PI/2),2.0*M_PI);
    laser_x = (cos(robot_rad)*laser_reads_.ranges[index]);
    laser_y = (sin(robot_rad)*laser_reads_.ranges[index]);

    return std::make_tuple(laser_x,laser_y);
}

void himm_inc(int grid_map[][4000], int robot_cell_x, int robot_cell_y, int laser_cell_x, int laser_cell_y, bool inf){
    int delta_x, delta_y, precision, precision2, xy2, x, y, xf, step_x, step_y;
    
    delta_x = laser_cell_x - robot_cell_x;
    delta_y = laser_cell_y - robot_cell_y;

    x = robot_cell_x;
    y = robot_cell_y;

    if (delta_x < 0) {
        delta_x = -delta_x;
        step_x = -1;
    } else {
        step_x = 1;
    }

    if (delta_y < 0) {
        delta_y = -delta_y;
        step_y = -1;
    } else {
        step_y = 1;
    }

    if (abs(delta_x) > abs(delta_y)) {
        precision = 2 * delta_y - delta_x;
        precision2 = 2 * delta_y;
        xy2 = 2 * (delta_y - delta_x);

        while(x != laser_cell_x){

            x += step_x;
            
            if (precision < 0) {
                precision += precision2;
            } else {
                y += step_y;
                precision += xy2;
            }
            
            grid_map[x][y] -= 7;
            if (grid_map[x][y] < 0) {
                grid_map[x][y] = 0;
            }         
        }
    } else {
        precision = 2 * delta_x - delta_y;
        precision2 = 2 * delta_x;
        xy2 = 2 * (delta_x - delta_y);

        while(y != laser_cell_y){
            y += step_y;
            
            if (precision < 0) {
                precision += precision2;
            } else {
                x += step_x;
                precision += xy2;
            }
            
            grid_map[x][y] -= 7;
            if (grid_map[x][y] < 0) {
                grid_map[x][y] = 0;
            }         
        }
    }

    if (!inf){
        grid_map[laser_cell_x][laser_cell_y] += 21;
        if (grid_map[laser_cell_x][laser_cell_y] > 100) {
            grid_map[laser_cell_x][laser_cell_y] = 100;
        }
    }
    
}

void basefootprintToLaserTF(){
    tf::TransformListener tf_listerner;
    tf::StampedTransform tf_trans;

    try {
        tf_listerner.waitForTransform("husky1_tf/base_footprint","husky1_tf/base_laser",ros::Time::now(),ros::Duration(3.0));
        tf_listerner.lookupTransform("husky1_tf/base_footprint","husky1_tf/base_laser",ros::Time::now(),tf_trans);
        // cout << "<<<<<<<<<<<<<<<<<<<<<<<<<ROBOT POSE>>>>>>>>>>>>>>>>>>>>>>>" << endl;
        // cout << "X: " << robot_pose[0] << " Y: " << robot_pose[1] << endl;
        // cout << "<<<<<<<<<<<<<<<<<<<<<<<<<TRANSFORM POSE>>>>>>>>>>>>>>>>>>>>>>>" << endl;
        // cout << "X: " << robot_pose[0] + tf_trans.getOrigin().x() << " Y: " << robot_pose[1] + tf_trans.getOrigin().y() << " Z: " << tf_trans.getOrigin().z() << endl;
        laser_pose[0] = robot_pose[0] + tf_trans.getOrigin().x();
        laser_pose[1] = robot_pose[1] + tf_trans.getOrigin().y();
    }catch(tf2::TransformException &ex){
        ROS_WARN("%s",ex.what());
    } 
}

nav_msgs::OccupancyGrid update_map(nav_msgs::OccupancyGrid map){

    int cell_x, cell_y, laser_cell_x, laser_cell_y;
    float odom_laser_x, odom_laser_y;

    int map_width2 = map.info.width;
    int map_height2 = map.info.height;

    std::tie(cell_x, cell_y) = odom2cell(laser_pose[0],laser_pose[1]);

    if (laser_reads_.ranges.size() > 0) {

        for (int i = 0; i < laser_reads_.ranges.size(); i++) {
            std::tie(odom_laser_x,odom_laser_y) = getLaserPosition(i);
            std::tie(laser_cell_x,laser_cell_y) = odom2cell(odom_laser_x+laser_pose[0],odom_laser_y+laser_pose[1]);

            if ((laser_cell_x >= 0 && laser_cell_x < map_width2) && (laser_cell_y >= 0 && laser_cell_y < map_height2)) {
                bool flag_inf = false;
                if (laser_reads_.ranges[i] != INFINITY){ 
                    himm_inc(map_grid, cell_x, cell_y, laser_cell_x, laser_cell_y, flag_inf);
                }else{
                    flag_inf = true;
                    himm_inc(map_grid, cell_x, cell_y, laser_cell_x, laser_cell_y, flag_inf);
                }
            }
        }
    }
    
    for(int x = 0; x < map_width2; x++){
        int multi = x * map_width2;
        for(int y = 0; y < map_height2; y++){
            int index = y + multi;
            map.data[index] = map_grid[x][y];
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

    nav_msgs::OccupancyGrid map_out;

    ros::Subscriber laser_sub = node.subscribe<sensor_msgs::LaserScan>("/husky1/scan",10,laserscan_callback);
    ros::Subscriber odom_sub = node.subscribe("/husky1/odometry/filtered",1000,odom_callback);

    ros::Publisher map_pub = node.advertise<nav_msgs::OccupancyGrid>("/map_out",10);

    ros::Subscriber grid_map = node.subscribe("/map",1,grid_map_callback);

    bool setup = true;
    ros::Rate rate(10);
    while(ros::ok()){
         
        if (map_out_.info.height > 0) {
            if (setup) {
                basefootprintToLaserTF();
                create_vector_map(map_out_.info.height, map_out_.info.width);
                setup = false;
                ros::spinOnce();
                rate.sleep();
            } else{
                basefootprintToLaserTF();
                map_out = update_map(map_out_);
                map_pub.publish(map_out);
                ros::spinOnce();
                rate.sleep();
            }
        } else {
            ros::spinOnce();
            rate.sleep();
        }
    }

    return 0;
}
