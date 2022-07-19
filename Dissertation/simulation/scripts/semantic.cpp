#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "ros/ros.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "nav_msgs/OccupancyGrid.h"

using namespace std;

nav_msgs::OccupancyGrid grid_map;
float object_pos_x = 0, object_pos_y = 0;


void darknet_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
    /*
        Ajustar para os diferentes tipos de objetos e os valores que eles irão representar no mapa
    */
    
    for (int x = 0; x < msg->bounding_boxes.size(); x++) {
        if (msg->bounding_boxes[x].Class == "suitcase") {
            cout << "SUITCASE | PROBABILITY: " << msg->bounding_boxes[x].probability << endl;
            cout << " X_MIN: " << msg->bounding_boxes[x].xmin << " X_MAX: " << msg->bounding_boxes[x].xmax << " X_MED: " << (msg->bounding_boxes[x].xmax - msg->bounding_boxes[x].xmin)/2 << endl;
            cout << " Y_MIN: " << msg->bounding_boxes[x].ymin << " Y_MAX: " << msg->bounding_boxes[x].ymax << " Y_MED: " << (msg->bounding_boxes[x].ymax - msg->bounding_boxes[x].ymin)/2 << endl;            
            object_pos_x = (msg->bounding_boxes[x].xmax - msg->bounding_boxes[x].xmin)/2;
            object_pos_y = (msg->bounding_boxes[x].ymax - msg->bounding_boxes[x].ymin)/2;
        }
    }
}

void grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg){
    grid_map.header.frame_id = map_msg->header.frame_id;
    grid_map.header.seq = map_msg->header.seq;
    grid_map.header.stamp = map_msg->header.stamp;
    grid_map.info.resolution = map_msg->info.resolution;
    grid_map.info.origin = map_msg->info.origin;
    grid_map.info.height = map_msg->info.height;
    grid_map.info.width = map_msg->info.width;
    grid_map.data = map_msg->data;
}

void map_update(){
    int width, heigth;
    
    width = grid_map.info.width;
    heigth = grid_map.info.height;

    // int map_data[width*heigth];

    // for (int x = 0; x < width*heigth; x++) {
    //     map_data[x] = grid_map.data[x];
    // }

    /*
        Ajustar este ponto para que o objeto detectado seja colocado no map_out
        Necessário fazer o cálculo da posição do objeto a partir da posição do robô 
        e colocar isso no mapa;
    */

    // for(int x = 0; x < width; x++){
    //     int multi = x * width;
    //     for(int y = 0; y < height; y++){
    //         int index = y + multi;
    //         map.data[index] = map_grid[x][y];
    //     }
    // }

}

int main(int argc, char **argv){

    ros::init(argc, argv, "semantic");
    ros::NodeHandle node;

    ros::Subscriber dark_sub = node.subscribe("/darknet_ros/bounding_boxes", 1000, darknet_callback);
    ros::Subscriber grid_map = node.subscribe("/map_out", 1000, grid_callback);

    ros::Rate rate(10);

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}