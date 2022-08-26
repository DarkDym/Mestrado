#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
// #include <string>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include "ros/ros.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/ObjectCount.h"
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

typedef struct {
    float x,y,z;
} Point3D;

nav_msgs::OccupancyGrid grid_map_;
cv_bridge::CvImageConstPtr cv_ptr_;
cv_bridge::CvImagePtr img_out_;
std::vector<Point3D> depthCloud_;
float cx,cy,fx,fy;
int IMG_WIDTH, IMG_HEIGTH;

const int map_width = 4000;
const int map_height = 4000;
const float map_resolution = 0.05;
const float map_origin_x = -107.00;
const float map_origin_y = -100.00; 

const int VASE_VALUE = 110;
const int BICYCLE_VALUE = 130;
const int PERSON_VALUE = 160;
const int SUITCASE_VALUE = 240;
const float HFOV_RAD = 1.5184351666666667;
const float CROP_SCALE = 4;

float ROBOT_POSE_[3];
bool can_publish = false;
int map_[4000][4000];

float camera_pose[2];

bool setup_map = true;
bool objectInMap_ = false;
int count_objects_ = 0;

std::vector<sensor_msgs::PointCloud2> depthCloudVec_;
sensor_msgs::PointCloud2 depthCloudROS;


//----------------------------------------------------------------------------------------------------------
bool mission_finished_;
std_msgs::Bool ros_finished_marking_;
// std::vector<std::tuple<int,int,int,int>> box_objects_;
vector<vector<int> > box_teste;
vector<std::string> box_class;
//----------------------------------------------------------------------------------------------------------

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
            if (map_[x][y] != SUITCASE_VALUE || map_[x][y] != PERSON_VALUE) {
                map_[x][y] = grid_map_.data[index];
            }else{
                cout << "------------------------------------------------------------------------------" << endl;
                cout << "COPY_MAP" << endl;
                cout << "&&&&&&&&&&&&&&&&&&&&&AQUI TEM ALGUM OBJETO DA CLASSE QUE ESTOU COLOCANDO NO MAPA!!!!!!!!!!!!!!!!!!!!!!!!11" << endl;
                cout << "VALOR QUE TA DENRTO DA COISA: " << map_[x][y] << endl;
                cout << "------------------------------------------------------------------------------" << endl;
            }
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

int matrix2vectorIndex(int cell_x, int cell_y, int map_width_){
    return cell_x + cell_y * map_width_; 
}

std::tuple<int,int> findSomeDepthValue(int x_i, int x_f, int y_i, int y_f, cv_bridge::CvImageConstPtr cv_ptr_in){
    float depth;
    cv_bridge::CvImageConstPtr cv_ptr_aux = cv_ptr_in;
    if (cv_ptr_aux){
        for (int x = x_i; x < x_f; x++) {
            for (int y = y_i; y < y_f; y++) { 
                depth = cv_ptr_aux->image.at<float>(y,x);
                if (!isnan(depth)) {
                    return std::make_tuple(y,x);
                } 
            }
        }
    }
    return std::make_tuple(-1,-1);
}

float meanDepthValue(int x_i, int x_f, int y_i, int y_f, cv_bridge::CvImageConstPtr cv_ptr_in){
    float depth = 0;
    int cont = 0;
    float acumulator = 0;
    vector<float> depth_vec;
    int mean = 0;
    cv_bridge::CvImageConstPtr cv_ptr_aux = cv_ptr_in;
    
    if (cv_ptr_aux){
        for (int x = x_i; x < x_f; x++) {
            for (int y = y_i; y < y_f; y++) {
                img_out_->image.at<float>(y,x) = 0;
                depth = cv_ptr_aux->image.at<float>(y,x);                
                if (!isnan(depth)) {
                    depth_vec.emplace_back(depth);
                    acumulator += depth;
                    cont++;
                } 
            }
        }
    }

    if (!depth_vec.empty()) {
        sort(depth_vec.begin(),depth_vec.end());
        if (depth_vec.size() % 2 == 0) {
            mean = depth_vec.size()/2;
        }else {
            mean = depth_vec.size()/2;
        }
    }

    if (cont != 0){
        if (depth_vec.size() % 2 == 0) {
            return  (depth_vec.at(mean+1) + depth_vec.at(mean))/2;
        } else {
            return depth_vec.at(mean+1);
        }
    } else {
        return 0;
    }
    
}
//--------------------------------------------------------------------------------------------------------------
void checkGridForValue(){
    cout << "ENTREI NA VERIFICACAO!!!!!" << endl;
    for(int x = 0; x < map_width; x++){
        int multi = x * map_width;
        for(int y = 0; y < map_height; y++){
            int index = y + multi;
            if (map_[x][y] == SUITCASE_VALUE || map_[x][y] == PERSON_VALUE) {
                cout << "ACHEI UM DOS OBJETOS QUE ESTAVA PROCURANDO!!!!" << map_[x][y] << endl;
            }
        }
    }
}
//--------------------------------------------------------------------------------------------------------------

void grid_update(){
    for(int x = 0; x < map_width; x++){
        int multi = x * map_width;
        for(int y = 0; y < map_height; y++){
            int index = y + multi;
            if (map_[x][y] != SUITCASE_VALUE || map_[x][y] != PERSON_VALUE) {
                grid_map_.data[index] = map_[x][y];
            }else{
                cout << "------------------------------------------------------------------------------" << endl;
                cout << "GRID_UPDATE" << endl;
                cout << "*****************AQUI TEM ALGUM OBJETO DA CLASSE QUE ESTOU COLOCANDO NO MAPA!!!!!!!!!!!!!!!!!!!!!!!!11" << endl;
                cout << "VALOR DO QUE EU COMPAREI: " << map_[x][y] << endl;
                cout << "------------------------------------------------------------------------------" << endl;
            }
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

    int radius = 1;
    for (int y = cell_y - radius; y < cell_y + radius; y++) {
        for (int x = cell_x - radius; x < cell_x + radius; x++) {
            if ((x >= 0 && x < map_width) && (y >= 0 && y < map_height)) {
                map_[x][y] = cell_value;
            }
        }
    }

    can_publish = true;
}


/*
Implementar esta função para escanear as marcações que estão no
mapa semântico e dever ser apagados do mesmo caso nenhum objeto exista no local. Neste momento estou
utilizando a posição da câmera do robô, contudo, é melhor conseguir
a posição que fica mais próximo do início da pointcloud.
*/
void scanForObejctsInMap(){
    int cell_x, cell_y;
    int radius = 50;

    tie(cell_x, cell_y) = odom2cell(camera_pose[0], camera_pose[1]);

    for (int y = cell_y; y < cell_y + radius; y++) {
        for (int x = cell_x - radius; x < cell_x + radius; x++) {
            if ((x >= 0 && x < map_width) && (y >= 0 && y < map_height)) {
                if (map_[x][y] == SUITCASE_VALUE || map_[x][y] == PERSON_VALUE) {
                    map_[x][y] = 0;
                }
            }
        }
    }
}

void basefootprintToCameraTF(){
    tf::TransformListener tf_listerner;
    tf::StampedTransform tf_trans;

    try {
        tf_listerner.waitForTransform("husky1_tf/base_footprint","husky1_tf/camera_realsense",ros::Time::now(),ros::Duration(3.0));
        tf_listerner.lookupTransform("husky1_tf/base_footprint","husky1_tf/camera_realsense",ros::Time::now(),tf_trans);
        camera_pose[0] = ROBOT_POSE_[0] + tf_trans.getOrigin().x();
        camera_pose[1] = ROBOT_POSE_[1] + tf_trans.getOrigin().y();
    }catch(tf2::TransformException &ex){
        ROS_WARN("%s",ex.what());
    } 
}

std::tuple<float,float> cloudTransform(int px, int py){
    
    int index = matrix2vectorIndex(px,py,depthCloudROS.width);
    
    return std::make_tuple(depthCloud_[index].x,depthCloud_[index].y);
}

void check_object_position(float depth, int object_pos_x, int object_pos_y, int cell_value){
    // float odom_x, odom_y;

    // double f = (IMG_WIDTH / 2.0) / tan(HFOV_RAD / 2.0);
    // double object_angle = atan(object_pos_x / f);
    // double center_angle = atan((int)(IMG_WIDTH/2) / f);
    // double max_angle = atan(IMG_WIDTH / f);
    // double max_angle_img = HFOV_RAD;
    // double phi_world = (M_PI - max_angle_img)/2;
    // double correction = 0;

    float object_x, object_y;

    std::tie(object_x,object_y) = cloudTransform(object_pos_x,object_pos_y);

    // if (cell_value == PERSON_VALUE) {
    //     double ang;
        
    //     if (object_angle == (max_angle_img/2)) {
    //         correction = 0;
    //     } else {
    //         correction = max_angle_img/2 - object_angle;
    //     }

    //     ang = ROBOT_POSE_[2] + correction;
    //     odom_x = camera_pose[0] + cos(ang) * depth;
    //     odom_y = camera_pose[1] + sin(ang) * depth;
    // } else {
    //     double ang;
        

    //     if (object_angle == (max_angle_img/2)) {
    //         correction = 0;
    //     } else {
    //         correction = max_angle_img/2 - object_angle;
    //     }

    //     ang = ROBOT_POSE_[2] + correction;
    //     odom_x = camera_pose[0] + cos(ang) * depth;
    //     odom_y = camera_pose[1] + sin(ang) * depth;
    // }

    map_update(object_x,object_y, cell_value);
}

void objectInSemanticMap(int object_pos_x, int object_pos_y){
    
    int cell_x, cell_y;

    tie(cell_x, cell_y) = odom2cell(object_pos_x, object_pos_y);
    
    int radius = 10;
    for (int y = cell_y - radius; y < cell_y + radius; y++) {
        for (int x = cell_x - radius; x < cell_x + radius; x++) {
            if ((x >= 0 && x < map_width) && (y >= 0 && y < map_height)) {
                if (map_[x][y] == SUITCASE_VALUE || map_[x][y] == PERSON_VALUE || map_[x][y] == VASE_VALUE || map_[x][y] == BICYCLE_VALUE) {
                    objectInMap_ = true;
                    break;
                } else {
                    objectInMap_ = false;
                }
            }
        }
        if (objectInMap_) {
            break;
        }
    }
}

void darknet_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){

    int screen_position_depth_x,screen_position_depth_y; 

    box_teste.clear();
    for (int x = 0; x < msg->bounding_boxes.size(); x++){
        vector<int> t;
        t.push_back(msg->bounding_boxes[x].xmin);
        t.push_back(msg->bounding_boxes[x].xmax);
        t.push_back(msg->bounding_boxes[x].ymin);
        t.push_back(msg->bounding_boxes[x].ymax);
        box_teste.push_back(t);
        box_class.push_back(msg->bounding_boxes[x].Class);
    }
    
    // for (const auto &i : box_objects_) {
    //     cout << "TESTE: " << get<0>(i) << " | " << get<1>(i) << " | " << get<2>(i) << " | " << get<3>(i) << endl;
    // }

    // for (int x = 0; x < msg->bounding_boxes.size(); x++) {
        
    //     //----------------------------------------------------------------------------------------------------------
    //     // box_objects_.push_back(std::make_tuple(msg->bounding_boxes[x].xmin,msg->bounding_boxes[x].xmax,msg->bounding_boxes[x].ymin,msg->bounding_boxes[x].ymax));
    //     // std::tuple<int,int,int,int> t = box_objects_.at(0);
        
    //     // cout << "TESTE: " << box_objects_.at(0) << endl;
    //     // cout << "VALOR: " << msg->bounding_boxes[x].xmin << endl;
    //     //----------------------------------------------------------------------------------------------------------

    //     int object_pos_x = 0, object_pos_y = 0;
    //     int bound_reduction_scale_x = (msg->bounding_boxes[x].xmax - msg->bounding_boxes[x].xmin)/CROP_SCALE;
    //     int bound_reduction_scale_y = (msg->bounding_boxes[x].ymax - msg->bounding_boxes[x].ymin)/CROP_SCALE;
    //     object_pos_x = (msg->bounding_boxes[x].xmax-bound_reduction_scale_x+msg->bounding_boxes[x].xmin+bound_reduction_scale_x)/2;
    //     object_pos_y = (msg->bounding_boxes[x].ymax-bound_reduction_scale_y+msg->bounding_boxes[x].ymin+bound_reduction_scale_y)/2;
    //     // object_pos_x = (msg->bounding_boxes[x].xmax + msg->bounding_boxes[x].xmin)/2;
    //     // object_pos_y = (msg->bounding_boxes[x].ymax + msg->bounding_boxes[x].ymin)/2;
    //     if (object_pos_x < IMG_WIDTH && object_pos_y < IMG_HEIGTH) {
    //         if (grid_map_.info.height > 0) {
    //             if (cv_ptr_){
    //                 float meanDepth = meanDepthValue(msg->bounding_boxes[x].xmin+bound_reduction_scale_x,msg->bounding_boxes[x].xmax-bound_reduction_scale_x,msg->bounding_boxes[x].ymin+bound_reduction_scale_y,msg->bounding_boxes[x].ymax-bound_reduction_scale_y,cv_ptr_);
    //                 float x_o = meanDepth * ((object_pos_x - cx)/fx);
    //                 float y_o = meanDepth * ((object_pos_y - cy)/fy);
    //                 if (!isnan(x_o)  && !isnan(y_o)) {
    //                     objectInSemanticMap(object_pos_x,object_pos_y);
    //                     if (!objectInMap_) {
    //                         if (msg->bounding_boxes[x].Class == "suitcase") {            
    //                             check_object_position(meanDepth,object_pos_x,object_pos_y,SUITCASE_VALUE);
    //                         } else if (msg->bounding_boxes[x].Class == "person") {
    //                             check_object_position(meanDepth,object_pos_x,object_pos_y,PERSON_VALUE);
    //                         } else if (msg->bounding_boxes[x].Class == "vase") {
    //                             check_object_position(meanDepth,object_pos_x,object_pos_y,VASE_VALUE);
    //                         } else if (msg->bounding_boxes[x].Class == "bicycle") {
    //                             check_object_position(meanDepth,object_pos_x,object_pos_y,BICYCLE_VALUE);
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //     } 
    // }
}

void boundToSemanticMap(){
    for (int x = 0; x < box_teste.size(); x++) {

        int object_pos_x = 0, object_pos_y = 0;
        int bound_reduction_scale_x = (box_teste[x][1] - box_teste[x][0])/CROP_SCALE;
        int bound_reduction_scale_y = (box_teste[x][3] - box_teste[x][2])/CROP_SCALE;
        object_pos_x = (box_teste[x][1] - bound_reduction_scale_x + box_teste[x][0] + bound_reduction_scale_x)/2;
        object_pos_y = (box_teste[x][3] - bound_reduction_scale_y + box_teste[x][2] + bound_reduction_scale_y)/2;

        if (object_pos_x < IMG_WIDTH && object_pos_y < IMG_HEIGTH) {
            if (grid_map_.info.height > 0) {
                if (cv_ptr_){
                    float meanDepth = meanDepthValue(box_teste[x][0] + bound_reduction_scale_x, box_teste[x][1] - bound_reduction_scale_x, box_teste[x][2] + bound_reduction_scale_y, box_teste[x][3] - bound_reduction_scale_y,cv_ptr_);
                    float x_o = meanDepth * ((object_pos_x - cx)/fx);
                    float y_o = meanDepth * ((object_pos_y - cy)/fy);
                    if (!isnan(x_o)  && !isnan(y_o)) {
                        objectInSemanticMap(object_pos_x,object_pos_y);
                        if (!objectInMap_) {
                            if (box_class[x] == "suitcase") {            
                                check_object_position(meanDepth,object_pos_x,object_pos_y,SUITCASE_VALUE);
                            } else if (box_class[x] == "person") {
                                check_object_position(meanDepth,object_pos_x,object_pos_y,PERSON_VALUE);
                            } else if (box_class[x] == "vase") {
                                check_object_position(meanDepth,object_pos_x,object_pos_y,VASE_VALUE);
                            } else if (box_class[x] == "bicycle") {
                                check_object_position(meanDepth,object_pos_x,object_pos_y,BICYCLE_VALUE);
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
            if (grid_map_.data[x] != SUITCASE_VALUE || grid_map_.data[x] != PERSON_VALUE || grid_map_.data[x] != VASE_VALUE || grid_map_.data[x] != BICYCLE_VALUE) {
                grid_map_.data[x] = map_msg->data[x];
            }else{
                cout << "------------------------------------------------------------------------------" << endl;
                cout << "GRID_CALLBACK" << endl;
                cout << "AQUI TEM ALGUM OBJETO DA CLASSE QUE ESTOU COLOCANDO NO MAPA!!!!!!!!!!!!!!!!!!!!!!!!11" << endl;
                cout << "VALOR QUE TA NA VARIAVEL: " << grid_map_.data[x] << endl;
                cout << "------------------------------------------------------------------------------" << endl;
            }
        }
        if (!can_publish){
            copy_map();
        }
    }
}

void depth_img_callback(const sensor_msgs::Image::ConstPtr& depth_msg){    
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
      img_out_ = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv_ptr_ = cv_ptr;
}

void caminfo_callback(const sensor_msgs::CameraInfoConstPtr& caminfo_msg){
    cx = caminfo_msg->K[2];
    cy = caminfo_msg->K[5];
    fx = caminfo_msg->K[0];
    fy = caminfo_msg->K[4];
    IMG_WIDTH = caminfo_msg->width;
    IMG_HEIGTH = caminfo_msg->height;
}

void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg){
    depthCloudROS.header = cloud_msg->header;
    depthCloudROS.height = cloud_msg->height;
    depthCloudROS.width = cloud_msg->width;
    depthCloudROS.row_step = cloud_msg->row_step;
    depthCloudROS.point_step = cloud_msg->point_step;
    depthCloudROS.fields = cloud_msg->fields;
    depthCloudROS.data = cloud_msg->data;

    int depthPoints = depthCloudROS.height * depthCloudROS.width;
    int p = 0;
    geometry_msgs::TransformStamped transform;
    sensor_msgs::PointCloud2 cloud_transform;
    tf2_ros::Buffer* tf_buffer_ = new tf2_ros::Buffer;
    tf2_ros::TransformListener* listener = new tf2_ros::TransformListener(*tf_buffer_);
    
    depthCloud_.resize(depthPoints);

    try{
        transform = tf_buffer_->lookupTransform("map", "husky1_tf/camera_realsense_gazebo", ros::Time(0), ros::Duration(3.0));
        tf2::doTransform(depthCloudROS, cloud_transform, transform);
    } catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        ROS_WARN("%s", depthCloudROS.header.frame_id.c_str());
    }

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_transform, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_transform, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_transform, "z");
    // sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_transform, "r");
    // sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_transform, "g");
    // sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_transform, "b");

    // cout << "CHEGUEI ATE AQUI!!!!!!!!!!!!!!!!!!!1111" << endl;
    

    for (size_t i = 0; i < depthPoints; ++i, ++iter_x, ++iter_y, ++iter_z) {
        depthCloud_[i].x = *iter_x;
        depthCloud_[i].y = *iter_y;
        depthCloud_[i].z = *iter_z;
    }
}

void found_object_callback(const darknet_ros_msgs::ObjectCount& count_msg){
    // cout << "QUANTIDADE DE OBJETOS ENCONTRADOS: " << count_msg.count << endl;
    count_objects_ = count_msg.count;
    // cout << "QNT: " << count_objects_ << endl;
}

//----------------------------------------------------------------------------------------------------------
void finished_mission_callback(const std_msgs::Bool& finished_mission_msg){
    mission_finished_ = finished_mission_msg.data;
}
//----------------------------------------------------------------------------------------------------------

int main(int argc, char **argv){

    ros::init(argc, argv, "semantic");
    ros::NodeHandle node;

    init_map();

    ros::Subscriber odom_sub = node.subscribe("/husky1/odometry/filtered", 1000, robot_pos_callback);
    ros::Subscriber dark_sub = node.subscribe("/darknet_ros/bounding_boxes", 1000, darknet_callback);
    ros::Subscriber grid_map = node.subscribe("/map_out", 1, grid_callback);
    ros::Subscriber depth_img = node.subscribe("/husky1/realsense/depth/image_rect_raw", 1000, depth_img_callback);
    ros::Subscriber cam_info_sub = node.subscribe("/husky1/realsense/color/camera_info", 1, caminfo_callback);
    ros::Subscriber point_cloud_sub = node.subscribe("/husky1/realsense/depth/color/points", 1, point_cloud_callback);
    ros::Subscriber dkn_object_sub = node.subscribe("/darknet_ros/found_object", 1000, found_object_callback);

    /*
        ***************************EXPERIMENTAL******************************************************
        Parte experimental para considerar as marcações no semantic_map somente quando chegou no goal.
        Os goals são controlados pelo patrol.cpp, utilizar um par de tópicos, sendo eles: um para mandar o resultado da
        missão atual e outro para enviar o resultado da marcação do semantic_map e resumo da missão.

        /husky1/finished_mission
            type: bool
            description: Utilizado para receber/enviar mensagem de finalização de cada missão executada pelo robô.
            parent: partol.cpp
        /husky1/finished_marking
            type: bool
            description: Utilizado para receber/enviar mensagem de finalização do processo de marcação dos objetos
                         no Semantic_map.
            parent: semantic.cpp
    */
    //--------------------------------------------------------------------
    ros::Subscriber mission_finished_sub = node.subscribe("/husky1/finished_mission",1000,finished_mission_callback);
    ros::Publisher finished_marking_pub = node.advertise<std_msgs::Bool>("/husky1/finished_marking",10);
    //--------------------------------------------------------------------

    ros::Publisher map_pub = node.advertise<nav_msgs::OccupancyGrid>("/map_out_s",10);
    ros::Publisher img_out = node.advertise<sensor_msgs::Image>("/img_out",10);
    ros::Publisher stop_mission_pub = node.advertise<std_msgs::Bool>("/stop_mission",10);
    ros::Publisher resume_mission_pub = node.advertise<std_msgs::Bool>("resume_mission",10);

    ros::Rate rate(10);

    while(ros::ok()){
        if (cv_ptr_){
            checkGridForValue();
            if (mission_finished_){    
                if (count_objects_ > 0) {
                    boundToSemanticMap();
                    if (can_publish) {
                        basefootprintToCameraTF();
                        grid_update();
                        
                        map_pub.publish(grid_map_);
                        can_publish = false;
                        img_out_->header.frame_id = cv_ptr_->header.frame_id;
                        img_out_->header.seq = cv_ptr_->header.seq;
                        img_out_->header.stamp = cv_ptr_->header.stamp;
                        img_out_->encoding = cv_ptr_->encoding;
                        img_out.publish(img_out_->toImageMsg());
                        ros_finished_marking_.data = true;
                        finished_marking_pub.publish(ros_finished_marking_);
                    } else {
                        ros_finished_marking_.data = false;
                        finished_marking_pub.publish(ros_finished_marking_);
                    }
                } else {
                    scanForObejctsInMap();
                    ros_finished_marking_.data = true;
                    finished_marking_pub.publish(ros_finished_marking_);
                }
            } else {
                ros_finished_marking_.data = false;
                finished_marking_pub.publish(ros_finished_marking_);
            }
            
        }
            ros::spinOnce();
            rate.sleep();
    }

    return 0;
}