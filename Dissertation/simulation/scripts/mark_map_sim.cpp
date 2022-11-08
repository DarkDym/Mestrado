#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>  
#include <map>

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

#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/ObjectCount.h"

using namespace std;
using namespace Eigen;

typedef struct {
    int OBJECT_CLASS_VALUE;
    int OBJECT_WEIGHT_VALUE;
    int OBJECT_CELL_X;
    int OBJECT_CELL_Y;
}OBJECT_SCANNED;

const int VASE_VALUE = 110;
const int BICYCLE_VALUE = 130;
const int PERSON_VALUE = 160;
const int SUITCASE_VALUE = 240;
const int UNKWON_VALUE = 80;

int map_[4000][4000];
nav_msgs::OccupancyGrid grid_map_;
nav_msgs::OccupancyGrid marked_map_sub_;
nav_msgs::OccupancyGrid marked_map_;
bool setup_map = true;
fstream objects_file_;
vector<tuple<int,float,float>> object_goals_;
bool finished_marking_ = false;
bool mission_finished_ = false;
int count_objects_ = 0;

float ROBOT_POSE_[3];
float CAMERA_POSE_[3];
const float ANGLE_INCREASE = 0.002811917;
const int MAX_BEAMS = 540;
const float MAX_RANGE_CAM_DEPTH = 2.5;
const float HFOV_RAD = 1.5184351666666667;
vector<std::string> box_class;
bool already_scanned_ = false;
bool all_objects_analyzed_ = false;
fstream yolo_file_;
vector<tuple<string,int,int,int>> yolo_objects_;

const map<std::string, int> DARKNET_CLASSES = {{"person", 160},{"vase", 110},{"bicycle", 130},{"suitcase", 240},{"unkwon", 80}};

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

void open_object_file(){
    yolo_file_.open("./src/Mestrado/Dissertation/simulation/config/yolo_object_weights.txt");
    if (yolo_file_.is_open()) {
        cout << "FILE yolo_object_weights OPENED SUCCEFULLY!" << endl;
    } else {
        cout << "COULD NOT OPEN CHOOSEN FILE!" << endl;
    }
}

void trash_file(string file_name){
    if (remove(file_name.c_str()) != 0) {
        cout << "ERRO AO TENTAR DELETAR O ARQUIVO!!!" << endl;
    } else {
        cout << "ARQUIVO" << file_name << " DELETADO COM SUCESSO!!!" << endl;
    }
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

void read_yolo_file(){
    string line, line_aux, obj_name;
    bool first_cell = false;
    tuple<string,int,int,int> obj_info;
    vector<int> aux_values;
    int aux;
    while(getline(yolo_file_,line)){
        stringstream st(line);
        while(getline(st, line_aux, ';')){
            cout << line_aux << endl;
            if (!first_cell) {
                obj_name = line_aux;
                first_cell = true;
            } else {
                aux = stoi(line_aux);
                aux_values.emplace_back(aux);
            }
        }
        cout << "----------------------" << endl;
        first_cell = false;
        obj_info = make_tuple(obj_name,aux_values[0],aux_values[1],aux_values[2]);
        yolo_objects_.emplace_back(obj_info);
        aux_values.clear();
    }
}

void robot_pos_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odom_msg){
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

void basefootprintToCameraTF(){
    tf::TransformListener tf_listerner;
    tf::StampedTransform tf_trans;

    try {
        tf_listerner.waitForTransform("pioneer_tf/base_link","pioneer_tf/camera_realsense",ros::Time::now(),ros::Duration(3.0));
        tf_listerner.lookupTransform("pioneer_tf/base_link","pioneer_tf/camera_realsense",ros::Time::now(),tf_trans);
        CAMERA_POSE_[0] = ROBOT_POSE_[0] + tf_trans.getOrigin().x();
        CAMERA_POSE_[1] = ROBOT_POSE_[1] + tf_trans.getOrigin().y();
    }catch(tf2::TransformException &ex){
        ROS_WARN("%s",ex.what());
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

//HIMM FOR IMAGE SCAN
//Later: Change the name of the function to clarify what it is doing.
void himm_inc(int robot_cell_x, int robot_cell_y, int laser_cell_x, int laser_cell_y){
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
            if (map_[x][y] != 0 || map_[x][y] != 255 || map_[x][y] != 100) {
                map_[x][y] = 0;
            }
            // //---------------TESTE
            // map_[x][y] = 100;       
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
            
            if (map_[x][y] != 0 || map_[x][y] != 255 || map_[x][y] != 100) {
                map_[x][y] = 0;
            } 
            // //---------------TESTE
            // map_[x][y] = 100;           
        }
    }    
}

void scanForObejctsInMap(){
    int cell_x, cell_y;
    int odom_y,odom_x;
    float laser_x, laser_y,robot_rad;
    float beta = HFOV_RAD/2;

    tie(cell_x, cell_y) = odom2cell(CAMERA_POSE_[0], CAMERA_POSE_[1]);

    for (int ang = 0; ang < MAX_BEAMS; ang++) {
        robot_rad = remainder(ROBOT_POSE_[2]+((ang*ANGLE_INCREASE)-beta),2.0*M_PI);
        laser_x = (cos(robot_rad) * MAX_RANGE_CAM_DEPTH);
        laser_y = (sin(robot_rad) * MAX_RANGE_CAM_DEPTH);
        tie(odom_x,odom_y) = odom2cell(CAMERA_POSE_[0]+laser_x,CAMERA_POSE_[1]+laser_y);
        if ((odom_x >= 0 && odom_x < grid_map_.info.width) && (odom_y >= 0 && odom_y < grid_map_.info.height)) {
            himm_inc(cell_x,cell_y,odom_x,odom_y);
        }
    }
}

//****************************************************Adicionado 7/11 ****************************************************
/*
    É necessário realizar uma mudança na forma de analisar o
    peso de cada objeto que é verificado pelo sistema.
*/
int verify_dinamyc(int cell_value){
    string obj_name;
    int obj_value, obj_cclean, obj_dynamic;
    int obj_unkwon = 2;
    for (int x = 0; x < yolo_objects_.size(); x++) {
        tie(obj_name,obj_value,obj_cclean,obj_dynamic) = yolo_objects_[x];
        if (cell_value == obj_value) {
            return obj_cclean;
        }
    }
    return obj_unkwon;
    // if (cell_value == 110) {
    //     return 0;
    // } else if (cell_value == 160 || cell_value == 120) {
    //     return 1;
    // } else {
    //     return 2;
    // }    
}

//****************************************************Adicionado 1/11 ****************************************************
/*
    Precisa ser testado. Até o momento somente foi adicionado
    e condicionado para funcionar de acordo com o objetivo de retirar o 
    objeto que está sendo observado e o que está sendo lido do arquivo do 
    mapa.
*/

void clean_specific_cell_from_matrix(int robot_cell_x, int robot_cell_y, int laser_cell_x, int laser_cell_y, int cell_value){
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
        
            if (map_[x][y] == cell_value) {
                map_[x][y] = 0;
            }
            // //---------------TESTE
            // map_[x][y] = 100;       
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
            
            if (map_[x][y] == cell_value) {
                map_[x][y] = 0;
            } 
            // //---------------TESTE
            // map_[x][y] = 100;           
        }
    }    
}

void scanForObejctsInMap(int cell_value){
    int cell_x, cell_y;
    int odom_y,odom_x;
    float laser_x, laser_y,robot_rad;
    float beta = HFOV_RAD/2;

    tie(cell_x, cell_y) = odom2cell(CAMERA_POSE_[0], CAMERA_POSE_[1]);

    for (int ang = 0; ang < MAX_BEAMS; ang++) {
        robot_rad = remainder(ROBOT_POSE_[2]+((ang*ANGLE_INCREASE)-beta),2.0*M_PI);
        laser_x = (cos(robot_rad) * MAX_RANGE_CAM_DEPTH);
        laser_y = (sin(robot_rad) * MAX_RANGE_CAM_DEPTH);
        tie(odom_x,odom_y) = odom2cell(CAMERA_POSE_[0]+laser_x,CAMERA_POSE_[1]+laser_y);
        if ((odom_x >= 0 && odom_x < grid_map_.info.width) && (odom_y >= 0 && odom_y < grid_map_.info.height)) {
            clean_specific_cell_from_matrix(cell_x,cell_y,odom_x,odom_y,cell_value);
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

void marked_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg){
    marked_map_sub_.info.height = map_msg->info.height;
}

void finished_mission_callback(const std_msgs::Bool& finished_mission_msg){
    mission_finished_ = finished_mission_msg.data;
}

void found_object_callback(const darknet_ros_msgs::ObjectCount& count_msg){
    count_objects_ = count_msg.count;
}

void darknet_bounding_boxes_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){

    box_class.clear();
    for (int x = 0; x < msg->bounding_boxes.size(); x++){
        box_class.push_back(msg->bounding_boxes[x].Class);
    }
}

void all_objects_analyzed_callback(const std_msgs::Bool& all_objects_msg){
    all_objects_analyzed_ = all_objects_msg.data;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "mark_map_sim");
    ros::NodeHandle node;

    init_map();
    open_file();
    read_file();
    int cell;
    float px,py;
    open_object_file();
    read_yolo_file();
    trash_file("./teste.txt");
    

    ros::Subscriber grid_map = node.subscribe("/map", 1, grid_callback);
    ros::Subscriber mission_finished_sub = node.subscribe("/reached_mission_goal",1000,finished_mission_callback);
    ros::Subscriber dkn_object_sub = node.subscribe("/darknet_ros/found_object", 1000, found_object_callback);
    ros::Subscriber marked_map_sub = node.subscribe("/map_marked", 1, marked_map_callback);
    ros::Subscriber dark_sub = node.subscribe("/darknet_ros/bounding_boxes", 1000, darknet_bounding_boxes_callback);
    ros::Subscriber odom_sub = node.subscribe("/pioneer/amcl_pose", 1000, robot_pos_callback);
    ros::Subscriber all_objects_analyzed_sub = node.subscribe("/all_objects_analyzed", 1, all_objects_analyzed_callback);  

    ros::Publisher map_pub = node.advertise<nav_msgs::OccupancyGrid>("/map_marked",10);
    ros::Publisher object_demarked_pub = node.advertise<std_msgs::Bool>("/object_demarked_from_map",10);

    ros::Rate rate(10);

    int object_cont_control,cont=0;
    
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
                if (marked_map_sub_.info.height > 0) {
                    finished_marking_ = true;
                    object_cont_control = 0;
                    cout << "FINISHED MARKING THE OBJECTS IN MAP!!!!!!" << endl;
                }
            } else {
                if (mission_finished_) {
                    if (object_cont_control <= object_goals_.size()-1) {
                        tie(cell,px,py) = object_goals_[object_cont_control];

                        if (count_objects_ <= 0) {
                            basefootprintToCameraTF();
                            scanForObejctsInMap();
                            std_msgs::Bool object_demarked;
                            object_demarked.data = true;
                            object_demarked_pub.publish(object_demarked);
                            object_cont_control++;
                        } else {
                            /*
                                Verificar se o objeto que está sendo analisado no arquivo
                                é o mesmo que sendo visto nesta posição ou mais além.
                                                        
                            */
                            if (!already_scanned_) {
                                int control_obj = 0;
                                for (int x = 0; x < box_class.size(); x++) {
                                    cout << "BOX_CLASS[" << x << "]: " << box_class[x] << endl;
                                    if (DARKNET_CLASSES.find(box_class[x]) != DARKNET_CLASSES.end()) {
                                        //Colocar uma flag de controle, para que o cell seja comparado com todos os possíveis objetos que foram detectados.
                                        if (cell != DARKNET_CLASSES.at(box_class[x])) {
                                            control_obj++;
                                            // int vcell = verify_dinamyc(cell);
                                            // if (vcell == 1) {
                                            //     basefootprintToCameraTF();
                                            //     scanForObejctsInMap(cell);
                                            // }                                                                                    
                                        }
                                    }
                                }

                                if (control_obj == box_class.size()-1) {
                                    int vcell = verify_dinamyc(cell);
                                    if (vcell == 1) {
                                        basefootprintToCameraTF();
                                        scanForObejctsInMap(cell);
                                    }
                                } 
                                
                                std_msgs::Bool object_demarked;
                                object_demarked.data = true;
                                object_demarked_pub.publish(object_demarked);
                                object_cont_control++;
                                already_scanned_ = true;
                                grid_update();
                                marked_map_.data = grid_map_.data;
                                map_pub.publish(marked_map_);
                            }

                            if (all_objects_analyzed_ && object_cont_control <= object_goals_.size()-1) {
                                object_cont_control++;
                            }
                        }
                    }
                } else {
                    already_scanned_ = false;
                }
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}