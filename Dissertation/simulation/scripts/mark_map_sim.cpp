#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>  

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

#include <librealsense2/rs.hpp>

#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace Eigen;

const int VASE_VALUE = 110;
const int BICYCLE_VALUE = 130;
const int PERSON_VALUE = 160;
const int SUITCASE_VALUE = 240;
const int UNKWON_VALUE = 80;

int map_[4000][4000];
nav_msgs::OccupancyGrid grid_map_;
nav_msgs::OccupancyGrid marked_map_;
bool setup_map = true;
fstream objects_file_;
vector<tuple<int,float,float>> object_goals_;
bool finished_marking_ = false;
bool mission_finished_ = false;
int count_objects_ = 0;

std::tuple<int,int> odom2cell(float odom_pose_x, float odom_pose_y){
    int i = odom_pose_x/grid_map_.info.resolution - grid_map_.info.origin.position.x/grid_map_.info.resolution;
    int j = odom_pose_y/grid_map_.info.resolution - grid_map_.info.origin.position.y/grid_map_.info.resolution;
    return std::make_tuple(j,i);
}

std::tuple<float,float> cell2odom(int cell_value_x, int cell_value_y){
    float x = (cell_value_x + grid_map_.info.origin.position.x/grid_map_.info.resolution) * grid_map_.info.resolution;
    float y = (cell_value_y + grid_map_.info.origin.position.y/grid_map_.info.resolution) * grid_map_.info.resolution;
    return std::make_tuple(x,y);
}

int matrix2vectorIndex(int cell_x, int cell_y, int map_width_){
    return cell_x + cell_y * map_width_; 
}

void open_file(){
    objects_file_.open("./objects_in_map.txt", ios::in);
    if (objects_file_.is_open()) {
        cout << "FILE OPENED SUCCEFULLY!" << endl;
    } else {
        cout << "COULD NOT OPEN CHOOSEN FILE!" << endl;
    }
}

void grid_update(){
    for(int x = 0; x < grid_map_.info.width; x++){
        int multi = x * grid_map_.info.width;
        for(int y = 0; y < grid_map_.info.height; y++){
            int index = y + multi;
                grid_map_.data[index] = map_[x][y];
        }
    }
}

void read_file(){
    string line, line_aux;
    int cell_value = 0;
    bool first_colum = false;
    tuple<int,float,float> aux_goals;
    vector<float> aux_pos;
    while(getline(objects_file_, line)){
        stringstream st(line);
        while(getline(st, line_aux, ';')){
            cout << line_aux << endl;
            if (!first_colum) {
                cell_value = stoi(line_aux);
                first_colum = true;    
            } else {
                aux_pos.emplace_back(stof(line_aux));
            }
        }
        cout << "----------------------" << endl;
        first_colum = false;
        aux_goals = make_tuple(cell_value,aux_pos[0],aux_pos[1]);
        object_goals_.emplace_back(aux_goals);
        cell_value = 0;
        aux_pos.clear();
    }
}

void map_update(float pos_x, float pos_y, int cell_value){
    int width, heigth;
    
    width = grid_map_.info.width;
    heigth = grid_map_.info.height;

    int cell_x, cell_y;

    tie(cell_x, cell_y) = odom2cell(pos_x, pos_y);
    int index;

    int radius = 3;
    for (int y = cell_y - radius; y < cell_y + radius; y++) {
        for (int x = cell_x - radius; x < cell_x + radius; x++) {
            if ((x >= 0 && x < grid_map_.info.width) && (y >= 0 && y < grid_map_.info.height)) {
                map_[x][y] = cell_value;
            }
        }
    }
}

void init_map(){
    for (int x = 0; x < 4000; x++) {
        for (int y = 0; y < 4000; y++) {
            map_[x][y] = -1;
        }
    }
}

void copy_map(){
    for(int x = 0; x < grid_map_.info.width; x++){
        int multi = x * grid_map_.info.width;
        for(int y = 0; y < grid_map_.info.height; y++){
            int index = y + multi;
            map_[x][y] = grid_map_.data[index];
        }
    }
}

void grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg){
    grid_map_.header.frame_id = map_msg->header.frame_id;
    grid_map_.header.seq = map_msg->header.seq;
    grid_map_.header.stamp = map_msg->header.stamp;
    grid_map_.info.resolution = map_msg->info.resolution;
    grid_map_.info.origin = map_msg->info.origin;
    grid_map_.info.height = map_msg->info.height;
    grid_map_.info.width = map_msg->info.width;
    if (setup_map) {
        grid_map_.data = map_msg->data;
        copy_map();
        setup_map = false;
    }
}

void finished_mission_callback(const std_msgs::Bool& finished_mission_msg){
    mission_finished_ = finished_mission_msg.data;
}

void found_object_callback(const darknet_ros_msgs::ObjectCount& count_msg){
    count_objects_ = count_msg.count;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "mark_map_sim");
    ros::NodeHandle node;

    init_map();
    open_file();
    cout << "SO PRA TESTAR SE TA FUNCIONANDO: " << endl;
    read_file();
    int cell;
    float px,py;
    

    ros::Subscriber grid_map = node.subscribe("/map", 1, grid_callback);
    ros::Subscriber mission_finished_sub = node.subscribe("/husky1/reached_mission_goal",1000,finished_mission_callback);
    ros::Subscriber dkn_object_sub = node.subscribe("/darknet_ros/found_object", 1000, found_object_callback);

    ros::Publisher map_pub = node.advertise<nav_msgs::OccupancyGrid>("/map_marked",10);

    ros::Rate rate(10);
    
    while(ros::ok()){
        if (grid_map_.info.width > 0) {
            if (!finished_marking_) {
                for (int x = 0; x < object_goals_.size(); x++) {
                    tie(cell,px,py) = object_goals_[x];
                    cout << "INDICE: " << x << " CELL: " << cell << " PX: " << px << " PY: " << py << endl;
                    map_update(px,py,cell);
                }   
                marked_map_.header.frame_id = grid_map_.header.frame_id;
                marked_map_.header.seq = grid_map_.header.seq;
                marked_map_.header.stamp = grid_map_.header.stamp;
                marked_map_.info.resolution = grid_map_.info.resolution;
                marked_map_.info.origin = grid_map_.info.origin;
                marked_map_.info.height = grid_map_.info.height;
                marked_map_.info.width = grid_map_.info.width;
                grid_update();
                marked_map_.data = grid_map_.data;
                map_pub.publish(marked_map_);
                finished_marking_ = true;
            } else {
                if (mission_finished_) {
                    /*
                        if objects_count <= 0
                            scanForObjectsInMap()
                    */
                }
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}