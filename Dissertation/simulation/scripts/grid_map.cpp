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


using namespace std;
using namespace Eigen;

class GridMap{
    private:
    public:
};

float robot_pose[3];
vector<float> laser_reads;
sensor_msgs::LaserScan laser_reads_;
float map_resolution = 0.05;
float map_origin_x = -100.00;
float map_origin_y = -100.00;
int map_height = 4000;
int map_width = 4000;
float map_grid[4000][4000];
float angle_increase = 0.00436;

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
    // cout << "YAW: " << euler[2] << endl;
}

void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_msg){
    robot_pose[0] = amcl_msg->pose.pose.position.x;
    robot_pose[1] = amcl_msg->pose.pose.position.y;
    Quaternionf q;
    q.x() = amcl_msg->pose.pose.orientation.x;
    q.y() = amcl_msg->pose.pose.orientation.y;
    q.z() = amcl_msg->pose.pose.orientation.z;
    q.w() = amcl_msg->pose.pose.orientation.w;
    cout << "X: " << amcl_msg->pose.pose.orientation.x;
    cout << " Y: " << amcl_msg->pose.pose.orientation.y;
    cout << " Z: " << amcl_msg->pose.pose.orientation.z;
    cout << " W: " << amcl_msg->pose.pose.orientation.w;
    auto euler = q.toRotationMatrix().eulerAngles(0,1,2);
    robot_pose[2] = euler[2];
    cout << " YAW: " << euler[2] << endl;
}

std::tuple<float,float> getLaserPosition(int index){
    if (laser_reads_.ranges[index] == INFINITY) return std::make_tuple(0,0);

    // float robot_rad = remainder((index*angle_increase)+robot_pose[2],2.0*M_PI);

    // float laser_x = (cos(robot_rad)*laser_reads_.ranges[index]);
    // float laser_y = (sin(robot_rad)*laser_reads_.ranges[index]);

    float robot_rad = remainder((robot_pose[2])+((index)*angle_increase),2.0*M_PI);
    float laser_x = (cos(robot_rad)*laser_reads_.ranges[index]);
    float laser_y = (sin(robot_rad)*laser_reads_.ranges[index]);

    return std::make_tuple(laser_x,laser_y);
}

void himm_inc(float grid_map[][4000], int robot_cell_x, int robot_cell_y, int laser_cell_x, int laser_cell_y, bool inf){
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

            if (delta_x < 0) {

            }
            x += step_x;
            
            if (precision < 0) {
                precision += precision2;
            } else {
                y += step_y;
                precision += xy2;
            }
            
            grid_map[x][y] -= 1;
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
            
            grid_map[x][y] -= 1;
            if (grid_map[x][y] < 0) {
                grid_map[x][y] = 0;
            }         
        }
    }

    
    if (!inf){
        grid_map[laser_cell_x][laser_cell_y] += 3;
        if (grid_map[laser_cell_x][laser_cell_y] > 100) {
            grid_map[laser_cell_x][laser_cell_y] = 100;
        }
    }
    
}

nav_msgs::OccupancyGrid update_map(nav_msgs::OccupancyGrid map){
    // cout << "ENTERING MAP UPDATE" << endl;
    int cell_x, cell_y, laser_cell_x, laser_cell_y;
    float odom_laser_x, odom_laser_y;
    std::tie(cell_x, cell_y) = odom2cell(robot_pose[0],robot_pose[1]);
    // cout << "ROBOT_POSE_X: " << robot_pose[0] << " ROBOT_POSE_Y: " << robot_pose[1] << endl;
    // cout << "CELL_X_ROBOT: " << cell_x << " CELL_Y_ROBOT: " << cell_y << endl;
    int radius = 3;
    for (int xi = cell_x - radius; xi < cell_x + radius; xi ++) {
        for (int yi = cell_y - radius; yi < cell_y + radius; yi ++) {
            if ((xi >= 0 && xi < map_width) && (yi >= 0 && yi < map_height)) {
                map_grid[xi][yi] = 100;
            }     
        }
    }

    // float robot_rad = remainder(robot_pose[2],2.0*M_PI);
    // float laser_x = (cos(robot_rad)*5);
    // float laser_y = (sin(robot_rad)*5);
    // std::tie(laser_cell_x,laser_cell_y) = odom2cell(laser_x+robot_pose[0],laser_y+robot_pose[1]);
    // radius = 1;
    // for (int xi = laser_cell_x - radius; xi < laser_cell_x + radius; xi ++) {
    //     for (int yi = laser_cell_y - radius; yi < laser_cell_y + radius; yi ++) {
    //         if ((xi >= 0 && xi < map_width) && (yi >= 0 && yi < map_height)) {
    //             map_grid[xi][yi] = 230;
    //         }     
    //     }
    // }
    // if ((cell_x >= 0 && cell_x < map_width) && (cell_y >= 0 && cell_y < map_height)) {
    //     map_grid[cell_x][cell_y] = 100;
    // }
    
    // cout << "QUANTIDADE DE LASERS: " << laser_reads_.ranges.size() << endl;
    if (laser_reads_.ranges.size() > 0) {
        for (int i = 0; i < laser_reads_.ranges.size(); i++) {
        //     std::tie(odom_laser_x,odom_laser_y) = getLaserPosition(i);
        //     cout << "ODOM_LASER_X: " << odom_laser_x << " ODOM_LASER_Y: " << odom_laser_y << endl;
        //     std::tie(laser_cell_x,laser_cell_y) = odom2cell(odom_laser_x+robot_pose[0],odom_laser_y+robot_pose[1]);
        //     cout << "LASER_CELL_X: " << laser_cell_x << " LASER_CELL_Y: " << laser_cell_y << endl;
        //     if ((laser_cell_x >= 0 && laser_cell_x < map_width) && (laser_cell_y >= 0 && laser_cell_y < map_height)) {
        //         // map_grid[laser_cell_x][laser_cell_y] = 100;
        //         himm_inc(map_grid, cell_x, cell_y, laser_cell_x, laser_cell_y);
        //     }
        // }
            radius = 3;
            // robot_rad = remainder(robot_pose[2]+(720*angle_increase),2.0*M_PI);
            // if (laser_reads_.ranges[0] != INFINITY) {
            //     laser_x = (cos(robot_rad)*laser_reads_.ranges[0]);
            //     laser_y = (sin(robot_rad)*laser_reads_.ranges[0]);
            // }
        std::tie(odom_laser_x,odom_laser_y) = getLaserPosition(i);
        std::tie(laser_cell_x,laser_cell_y) = odom2cell(odom_laser_x+robot_pose[0],odom_laser_y+robot_pose[1]);
        
        //     cout << "ODOM_LASER_X: " << odom_laser_x << " ODOM_LASER_Y: " << odom_laser_y << endl;
        //     cout << "ROBOT_POSE_X: " << robot_pose[0] << " ROBOT_POSE_Y: " << robot_pose[1] << "ROBOT_POSE_O: " << robot_pose[2] << endl;
        //     cout << "PLUS_POSE_X: " << odom_laser_x+robot_pose[0] << " PLUS_POSE_Y: " << odom_laser_y+robot_pose[1] << endl;
        //     std::tie(laser_cell_x,laser_cell_y) = odom2cell(odom_laser_x+robot_pose[0],odom_laser_y+robot_pose[1]);
            // cout << "LASER_CELL_X: " << laser_cell_x << " LASER_CELL_Y: " << laser_cell_y << endl;
            // cout << "CELL_X_ROBOT: " << cell_x << " CELL_Y_ROBOT: " << cell_y << endl;
            if ((laser_cell_x >= 0 && laser_cell_x < map_width) && (laser_cell_y >= 0 && laser_cell_y < map_height)) {
                // map_grid[laser_cell_x][laser_cell_y] = 100;
                bool flag_inf = false;
                if (laser_reads_.ranges[i] != INFINITY){ 
                    himm_inc(map_grid, cell_x, cell_y, laser_cell_x, laser_cell_y, flag_inf);
                    for (int xi = laser_cell_x - radius; xi < laser_cell_x + radius; xi ++) {
                        for (int yi = laser_cell_y - radius; yi < laser_cell_y + radius; yi ++) {
                            if ((xi >= 0 && xi < map_width) && (yi >= 0 && yi < map_height)) {
                                map_grid[xi][yi] = 200;
                            }
                        }
                    }
                }else{
                    flag_inf = true;
                    himm_inc(map_grid, cell_x, cell_y, laser_cell_x, laser_cell_y, flag_inf);
                }
            }
        }
    }
    
    
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
    nav_msgs::OccupancyGrid map_out;

    ros::Subscriber laser_sub = node.subscribe<sensor_msgs::LaserScan>("/husky1/scan",10,laserscan_callback);
    ros::Subscriber odom_sub = node.subscribe("/husky1/amcl_pose",1000,amcl_callback);
    // ros::Subscriber odom_sub = node.subscribe("/husky1/odometry/filtered",1000,odom_callback);

    ros::Publisher map_pub = node.advertise<nav_msgs::OccupancyGrid>("/map_out",10);
    
    // map_pub.publish(map);
    ros::Rate rate(10);
    while(ros::ok()){
        map_out = update_map(map);
        map_pub.publish(map_out);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
