#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


#include "ros/ros.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

#include <librealsense2/rs.hpp>

#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace Eigen;

nav_msgs::OccupancyGrid grid_map_;
cv_bridge::CvImageConstPtr cv_ptr_;
// sensor_msgs::CameraInfo camera_info_;
float cx,cy,fx,fy;

const int map_width = 4000;
const int map_height = 4000;
const float map_resolution = 0.05;
const float map_origin_x = -107.00;
const float map_origin_y = -100.00; 

const int SUITCASE_VALUE = 240;
const int PERSON_VALUE = 160;
const float HFOV_RAD = 1.21;
const int IMG_WIDTH = 640;
const int IMG_HEIGTH = 480;

const float focal_point = 337.2084410968044;

float ROBOT_POSE_[3];
bool can_publish = false;
int map_[4000][4000];

bool setup_map = true;

void init_map(){
    for (int x = 0; x < 4000; x++) {
        for (int y = 0; y < 4000; y++) {
            map_[x][y] = -1;
        }
    }
}

void copy_map(){
    for(int x = 0; x < map_width; x++){
        int multi = x * map_width;
        for(int y = 0; y < map_height; y++){
            int index = y + multi;
            map_[x][y] = grid_map_.data[index];
        }
    }
}

std::tuple<int,int> odom2cell(float odom_pose_x, float odom_pose_y){
    int i = odom_pose_x/map_resolution - map_origin_x/map_resolution;
    int j = odom_pose_y/map_resolution - map_origin_y/map_resolution;
    return std::make_tuple(j,i);
}

std::tuple<float,float> cell2odom(int cell_value_x, int cell_value_y){
    float x = (cell_value_x + map_origin_x/map_resolution) * map_resolution;
    float y = (cell_value_y + map_origin_y/map_resolution) * map_resolution;
    return std::make_tuple(x,y);
}

int matrix2vectorIndex(int cell_x, int cell_y){
    return cell_x + cell_y * map_width; 
}

std::tuple<int,int> findSomeDepthValue(int x_i, int x_f, int y_i, int y_f, cv_bridge::CvImageConstPtr cv_ptr_in){
    float depth;
    // int cont = 0;
    // float acumulator = 0;
    cv_bridge::CvImageConstPtr cv_ptr_aux = cv_ptr_in;
    if (cv_ptr_aux){
        cout << "RECEBI OS VALORES: " << x_i <<  " " << x_f << " " << y_i << " " << y_f << endl;
        for (int x = x_i; x < x_f; x++) {
            for (int y = y_i; y < y_f; y++) {
                // cout << "VALOR DOS INDICES: [" << x << " , " << y << "] : " << endl; //<< cv_ptr_aux->image.at<float>(x,y) << endl;
                // cout << "NORMALIZANDO PIXEL NO EIXO Y: " << 480-y << " | VALOR DO DEPTH COM Y NORMALIZADO: " << cv_ptr_aux->image.at<float>(x,480-y) << endl;  
                depth = cv_ptr_aux->image.at<float>(x,480-y);
                if (!isnan(depth)) {
                    // acumulator += depth;
                    // cont++;
                    cout << "RETORNANDO O VALOR DA TELA" << endl;
                    return std::make_tuple(x,480-y);
                } 
            }
        }
    }
    cout << "NAO FOI POSSIVEL ACHAR UM DEPTH COM OS VALORES RECEBIDOS" << endl;
    return std::make_tuple(-1,-1);
}

float meanDepthValue(int x_i, int x_f, int y_i, int y_f, cv_bridge::CvImageConstPtr cv_ptr_in){
    float depth;
    int cont = 0;
    float acumulator = 0;
    vector<float> depth_vec;
    cv_bridge::CvImageConstPtr cv_ptr_aux = cv_ptr_in;
    if (cv_ptr_aux){
        cout << "RECEBI OS VALORES: " << x_i <<  " " << x_f << " " << y_i << " " << y_f << endl;
        for (int x = x_i; x < x_f; x++) {
            for (int y = y_i; y < y_f; y++) { 
                depth = cv_ptr_aux->image.at<float>(x,480-y);
                depth_vec.emplace_back(depth);
                if (!isnan(depth)) {
                    acumulator += depth;
                    cont++;
                } 
            }
        }
    }

    sort(depth_vec.begin(),depth_vec.end());

    if (depth_vec.size() % 2 == 0) {
        cout << "PAR" << endl;
        cout << "TAMANHO DO VECTOR: " << depth_vec.size() << endl;
        cout << "METADE DO VECTOR: " << (int)(depth_vec.size()/2) << endl;
        int mean = depth_vec.size()/2;
        cout << "MEDIANA: " << (depth_vec.at(mean+1) + depth_vec.at(mean))/2 << endl;
    }else {
        cout << "IMPAR" << endl; 
        cout << "TAMANHO DO VECTOR: " << depth_vec.size() << endl;
        cout << "METADE DO VECTOR: " << (int)(depth_vec.size()/2) << endl;
        int mean = depth_vec.size()/2;
        cout << "MEDIANA: " << depth_vec.at(mean+1) << endl;
    }

    if (cont != 0){
        cout << "MEAN DEPTH VALUE: " << acumulator/cont << endl;
        return acumulator/cont;
    } else {
        return -1;
    }
    
}

void grid_update(){
    for(int x = 0; x < map_width; x++){
        int multi = x * map_width;
        for(int y = 0; y < map_height; y++){
            int index = y + multi;
            grid_map_.data[index] = map_[x][y];
        }
    }
}

void map_update(float pos_x, float pos_y, int cell_value){
    int width, heigth;
    
    width = grid_map_.info.width;
    heigth = grid_map_.info.height;

    int cell_x, cell_y;

    tie(cell_x, cell_y) = odom2cell(pos_x, pos_y);
    int index;

    // cout << "CHEGUEI ATE AQUI!!!!" << endl;

    int radius = 1;
    for (int y = cell_y - radius; y < cell_y + radius; y++) {
        for (int x = cell_x - radius; x < cell_x + radius; x++) {
            if ((x >= 0 && x < map_width) && (y >= 0 && y < map_height)) {
                // index = matrix2vectorIndex(x,y);
                map_[x][y] = cell_value;
                // grid_map_.data[index] = cell_value;
            }
        }
    }

    can_publish = true;
    // for(int x = 0; x < map_width; x++){
    //     int multi = x * map_width;
    //     for(int y = 0; y < map_height; y++){
    //         index = y + multi;
    //         grid_map_.data[index] = map_[x][y];
    //     }
    // }
    

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

void check_object_position(int depth, int object_pos_x, int cell_value){
    float odom_x, odom_y;
    // odom_x = object_pos_x;
    // odom_y = object_pos_y;
    double f = (IMG_WIDTH / 2.0) / tan(HFOV_RAD / 2.0);
    double object_angle = atan(object_pos_x / f);
    // float object_angle = atan(object_pos_x/depth);
    cout << "CENTER ANGLE:" << atan((int)(IMG_WIDTH/2) / f) << endl;
    if (cell_value == PERSON_VALUE) {
        cout << "PERSON ANGLE : " << object_angle << endl;
    } else {
        cout << "SUITCASE ANGLE : " << object_angle << endl;
    }
    
    odom_x = ROBOT_POSE_[0] + cos(ROBOT_POSE_[2]+object_angle) * depth;
    odom_y = ROBOT_POSE_[1] + sin(ROBOT_POSE_[2]+object_angle) * depth;
    // copy_map();
    // int index = object_pos_x + test_depth_img.width * object_pos_y;    
    map_update(odom_x,odom_y, cell_value);
}

void darknet_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
    /*
        Ajustar para os diferentes tipos de objetos e os valores que eles irão representar no mapa
    */

    int screen_position_depth_x,screen_position_depth_y; 

    for (int x = 0; x < msg->bounding_boxes.size(); x++) {
        int object_pos_x = 0, object_pos_y = 0;
        if (msg->bounding_boxes[x].Class == "suitcase") {
            cout << "SUITCASE | PROBABILITY: " << msg->bounding_boxes[x].probability << endl;
            cout << " X_MIN: " << msg->bounding_boxes[x].xmin << " X_MAX: " << msg->bounding_boxes[x].xmax << " X_MED: " << (msg->bounding_boxes[x].xmax + msg->bounding_boxes[x].xmin)/2 << endl;
            cout << " Y_MIN: " << msg->bounding_boxes[x].ymin << " Y_MAX: " << msg->bounding_boxes[x].ymax << " Y_MED: " << (msg->bounding_boxes[x].ymax + msg->bounding_boxes[x].ymin)/2 << endl;            
            object_pos_x = (msg->bounding_boxes[x].xmax + msg->bounding_boxes[x].xmin)/2;
            object_pos_y = (msg->bounding_boxes[x].ymax + msg->bounding_boxes[x].ymin)/2;           
            cout << "CALCULADO" << endl;
            if (object_pos_x < 640 && object_pos_y < 480) {
                cout << "POSICAO DO OBJETO DENTRO DO FRAME!!!" << endl;
                if (grid_map_.info.height > 0) {
                    if (cv_ptr_){
                        cout << "VOU CALCULAR O DEPTH" << endl;
                        tie(screen_position_depth_x,screen_position_depth_y) = findSomeDepthValue(msg->bounding_boxes[x].xmin,msg->bounding_boxes[x].xmax,msg->bounding_boxes[x].ymin,msg->bounding_boxes[x].ymax,cv_ptr_);
                        float meanDepth = meanDepthValue(msg->bounding_boxes[x].xmin,msg->bounding_boxes[x].xmax,msg->bounding_boxes[x].ymin,msg->bounding_boxes[x].ymax,cv_ptr_);
                        if (screen_position_depth_x != -1 && screen_position_depth_y != -1) {
                            cout << "SUITCASE MEAN DEPTH VALUE AT (" << screen_position_depth_x << " , " << screen_position_depth_y << "): " << cv_ptr_->image.at<float>(screen_position_depth_x,screen_position_depth_y) << endl;
                            float depth = cv_ptr_->image.at<float>(screen_position_depth_x,screen_position_depth_y);
                            float x_o = meanDepth * ((object_pos_x - cx)/fx);
                            float y_o = meanDepth * ((object_pos_y - cy)/fy);
                            cout << "INTRINSICOS: CX: " << cx << " | CY: " << cy << " | FX: " << fx << " | FY: " << fy << endl;
                            if (!isnan(x_o)  && !isnan(y_o)) {
                                cout << "POSICAO SUITCASE: [" << meanDepth << " , " << x_o << "]" << endl;
                                cout << "POSICAO SUITCASE EM RELACAO ROBO: [" << ROBOT_POSE_[0]+meanDepth << " , " << ROBOT_POSE_[1]+x_o << "]" <<endl; 
                                check_object_position(meanDepth,object_pos_x, SUITCASE_VALUE);
                            }
                        }
                    }
                }
            }
            cout << "FINALIZEI A SUITCASE" << endl;
        } else if (msg->bounding_boxes[x].Class == "person") {
            cout << "PERSON | PROBABILITY: " << msg->bounding_boxes[x].probability << endl;
            cout << " X_MIN: " << msg->bounding_boxes[x].xmin << " X_MAX: " << msg->bounding_boxes[x].xmax << " X_MED: " << (msg->bounding_boxes[x].xmax + msg->bounding_boxes[x].xmin)/2 << endl;
            cout << " Y_MIN: " << msg->bounding_boxes[x].ymin << " Y_MAX: " << msg->bounding_boxes[x].ymax << " Y_MED: " << (msg->bounding_boxes[x].ymax + msg->bounding_boxes[x].ymin)/2 << endl;            
            object_pos_x = (msg->bounding_boxes[x].xmax + msg->bounding_boxes[x].xmin)/2;
            object_pos_y = (msg->bounding_boxes[x].ymax + msg->bounding_boxes[x].ymin)/2;
            if (object_pos_x < 640 && object_pos_y < 480) {
                if (grid_map_.info.height > 0) {
                    if (cv_ptr_){
                        tie(screen_position_depth_x,screen_position_depth_y) = findSomeDepthValue(msg->bounding_boxes[x].xmin,msg->bounding_boxes[x].xmax,msg->bounding_boxes[x].ymin,msg->bounding_boxes[x].ymax,cv_ptr_);
                        float meanDepth = meanDepthValue(msg->bounding_boxes[x].xmin,msg->bounding_boxes[x].xmax,msg->bounding_boxes[x].ymin,msg->bounding_boxes[x].ymax,cv_ptr_);
                        if (screen_position_depth_x != -1 && screen_position_depth_y != -1) {
                            cout << "PERSON MEAN DEPTH VALUE AT (" << screen_position_depth_x << " , " << screen_position_depth_y << "): " << cv_ptr_->image.at<float>(screen_position_depth_x,screen_position_depth_y) << endl;
                            float depth = cv_ptr_->image.at<float>(screen_position_depth_x,screen_position_depth_y);
                            float x_o = meanDepth * ((object_pos_x - cx)/fx);
                            float y_o = meanDepth * ((object_pos_y - cy)/fy);
                            cout << "INTRINSICOS: CX: " << cx << " | CY: " << cy << " | FX: " << fx << " | FY: " << fy << endl;
                            if (!isnan(x_o)  && !isnan(y_o)) {
                                cout << "POSICAO PERSON: [" << meanDepth << " , " << x_o << "]" << endl;
                                cout << "POSICAO PERSON EM RELACAO ROBO: [" << ROBOT_POSE_[0]+meanDepth << " , " << ROBOT_POSE_[1]+x_o << "]" <<endl; 
                                check_object_position(meanDepth,object_pos_x, PERSON_VALUE);
                            }
                        }
                    }
                }
            }
        }
    }
}

void robot_pos_callback(const nav_msgs::Odometry::ConstPtr& odom_msg){
    ROBOT_POSE_[0] = odom_msg->pose.pose.position.x;
    ROBOT_POSE_[1] = odom_msg->pose.pose.position.y;
    Quaternionf q;
    q.x() = odom_msg->pose.pose.orientation.x;
    q.y() = odom_msg->pose.pose.orientation.y;
    q.z() = odom_msg->pose.pose.orientation.z;
    q.w() = odom_msg->pose.pose.orientation.w;
    auto euler = q.toRotationMatrix().eulerAngles(0,1,2);
    ROBOT_POSE_[2] = euler[2];
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
    } else {
        for (int x = 0; x < map_msg->info.width*map_msg->info.height; x++) {
            // if (grid_map_.data[x] != 160 || grid_map_.data[x] != 240) {
                grid_map_.data[x] = map_msg->data[x];
            // }
        }
        copy_map();
    }
}

void depth_img_callback(const sensor_msgs::Image::ConstPtr& depth_msg){    
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
    //   cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
      cv_ptr = cv_bridge::toCvShare(depth_msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // ROS_INFO("Applying mask to depth image");
    // cv::Mat m = cv::Mat::zeros(cv_ptr->image.size(), CV_32FC1);
    // cv::Mat m = cv::Mat(cv_ptr->image.size(), cv_ptr->image.type());
    // cout << "TAMANHO: " << cv_ptr->image.size() << endl; 
    // cout << "TESTE: " << cv_ptr->image.at<float>(302,248) << endl;
    cv_ptr_ = cv_ptr;
}

void caminfo_callback(const sensor_msgs::CameraInfoConstPtr& caminfo_msg){
    // camera_info_ = caminfo_msg;
    cx = caminfo_msg->K[2];
    cy = caminfo_msg->K[5];
    fx = caminfo_msg->K[0];
    fy = caminfo_msg->K[4];
    cout << "INTRINSICOS: CX: " << cx << " | CY: " << cy << " | FX: " << fx << " | FY: " << fy << endl;
}



int main(int argc, char **argv){

    ros::init(argc, argv, "semantic");
    ros::NodeHandle node;

    init_map();

    ros::Subscriber odom_sub = node.subscribe("/husky1/odometry/filtered", 1000, robot_pos_callback);
    ros::Subscriber dark_sub = node.subscribe("/darknet_ros/bounding_boxes", 1000, darknet_callback);
    ros::Subscriber grid_map = node.subscribe("/map_out", 1, grid_callback);
    ros::Subscriber depth_img = node.subscribe("/husky1/realsense/depth/image_rect_raw", 1000, depth_img_callback);
    ros::Subscriber cam_info_sub = node.subscribe("/husky1/realsense/color/camera_info", 1, caminfo_callback);

    ros::Publisher map_pub = node.advertise<nav_msgs::OccupancyGrid>("/map_out_s",10);

    ros::Rate rate(10);

    while(ros::ok()){
        if (cv_ptr_){
            if (can_publish) {
                grid_update();
                map_pub.publish(grid_map_);
                can_publish = false;
            }
        }
            ros::spinOnce();
            rate.sleep();
    }

    return 0;
}