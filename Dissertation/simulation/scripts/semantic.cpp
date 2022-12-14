#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <fstream>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include "ros/ros.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/ObjectCount.h"
#include "nav_msgs/OccupancyGrid.h"
#include "map_msgs/OccupancyGridUpdate.h"
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
nav_msgs::OccupancyGrid lane_map_;
map_msgs::OccupancyGridUpdate lane_map_update_;
cv_bridge::CvImageConstPtr cv_ptr_;
cv_bridge::CvImagePtr img_out_;
std::vector<Point3D> depthCloud_;
float cx,cy,fx,fy;
int IMG_WIDTH, IMG_HEIGTH;

const int map_width = 4000;
const int map_height = 4000;
const float map_resolution = 0.05;
const float map_origin_x = -100.00;
const float map_origin_y = -100.00; 

const float HFOV_RAD = 1.5184351666666667;
const float CROP_SCALE = 4;

const int VASE_VALUE = 110;
const int BICYCLE_VALUE = 130;
const int PERSON_VALUE = 160;
const int SUITCASE_VALUE = 240;
const int UNKWON_VALUE = 80;

const float ANGLE_INCREASE = 0.002811917;
const int MAX_BEAMS = 540;
const int MAX_RANGE_CAM_DEPTH = 15;

float ROBOT_POSE_[3];
bool can_publish = false;
int map_[4000][4000];

float camera_pose[3];

bool setup_map = true;
bool objectInMap_ = false;
int count_objects_ = 0;

std::vector<sensor_msgs::PointCloud2> depthCloudVec_;
sensor_msgs::PointCloud2 depthCloudROS;


//----------------------------------------------------------------------------------------------------------
bool mission_finished_;
std_msgs::Bool ros_finished_marking_;
vector<vector<int> > box_teste;
vector<std::string> box_class;
bool all_objects_marked_ = false;
bool is_crowd_ = false;
bool already_mark_crowd_ = false;
bool crowd_published_ = false;
bool setup_lane_map_ = true;
bool t_lane_ = true;
int map_custom_[4000][4000];
int p_cont_ = 0;
int p_cont_aux_ = 0;
vector<tuple<int,int>> person_barrier_; 
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
            if (map_[x][y] != SUITCASE_VALUE && map_[x][y] != PERSON_VALUE && map_[x][y] != VASE_VALUE && map_[x][y] != BICYCLE_VALUE) {
                map_[x][y] = grid_map_.data[index];
            }
        }
    }
}

//Funções custom são utilizadas para a alteração do lane_map, está com esse nome pois estava tentando fazer genérica
//contudo neste momento não sei como posso fazer. Irei manter não-genérico para realizar os testes necessários o quanto antes.
void init_map_custom(){
    for (int x = 0; x < 4000; x++) {
        for (int y = 0; y < 4000; y++) {
            map_custom_[x][y] = -1;
        }
    }
}

void copy_map_custom(){
    for(int x = 0; x < map_width; x++){
        int multi = x * map_width;
        for(int y = 0; y < map_height; y++){
            int index = y + multi;
            // if (map_custom_[x][y] != PERSON_VALUE){
                map_custom_[x][y] = lane_map_.data[index];
            // }
        }
    }
}

void grid_update_custom(){
    for(int x = 0; x < map_width; x++){
        int multi = x * map_width;
        for(int y = 0; y < map_height; y++){
            int index = y + multi;
            lane_map_.data[index] = map_custom_[x][y];
        }
    }
}
//---------------------------------------------------------------------------------------------------------------------------------

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
    for(int x = 0; x < map_width; x++){
        int multi = x * map_width;
        for(int y = 0; y < map_height; y++){
            int index = y + multi;
            if (map_[x][y] == SUITCASE_VALUE || map_[x][y] == PERSON_VALUE || map_[x][y] == VASE_VALUE || map_[x][y] == BICYCLE_VALUE) {
                // cout << "ACHEI UM DOS OBJETOS QUE ESTAVA PROCURANDO!!!!" << map_[x][y] << endl;
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
            grid_map_.data[index] = map_[x][y];
        }
    }
}

/*
---FUNÇÃO QUE RECEBE O TIPO DE OBJETO E A SUA POSIÇÃO REFERENTE NO MAPA---
---ESTAS INFORMAÇÕES SERÃO UTILIZADAS PARA O ROBÔ SECUNDÁRIO QUE IRÁ FAZER A RETIRADA DE ITENS LEVES---
*/
void write_object_in_file(int cell_value, float pos_x, float pos_y){

    ofstream objects_map_file;
    objects_map_file.open("./objects_in_map.txt",ios::app);
    if (objects_map_file.is_open() && !all_objects_marked_) {
        cout << "CELL_VALUE:" << cell_value << ";POS_X:" << pos_x << ";POS_Y:" << pos_y << endl;
        objects_map_file << cell_value << ";" << pos_x << ";" << pos_y << endl;
        objects_map_file.close();
    }else{
        cout << "POR ALGUM MOTIVO O ARQUIVO NAO PODE SER ABERTO OU TODOS OS OBJETOS JA FORAM MARCADOS NO ARQUIVO!" << endl;
    }
}

/*
    ADICIONADO 24/11/22
    ESTAVA DENTRO DO MARK_MAP_SIM.CPP, CONTUDO COMO O HUSKY É QUE REALIZA A MARCAÇÃO
    FOI TRANSFERIDO PARA ESTE ARQUIVO.
*/
void person_count(int cell_x, int cell_y){
    /*
        --> RECEBER A CONTAGEM DE OBJETOS IDENTIFICADOS PELA YOLO;
        --> VERIFICAR SE EXISTE MAIS DO QUE X OBJETOS QUE SÃO PERSON;
        --> VERIFICAR SE A QUANTIDADE É MAIOR OU IGUAL A X;
        --> SE VERDADEIRO, REALIZAR UMA BUSCA POR PAREDES EM TODAS AS DIREÇÕES;
        --> ACHANDO AS PAREDES, REALIZAR O FECHAMENTO DAS MESMAS;
        --> REALIZAR UM NOVO GOAL ATRAS DA BARREIRA PARA VERIFICAR SE O MÉTODO FUNCIONA;
    */
    int corridor_threshold = 100;
    bool first_cell = false, blank_cell = false;
    int cell_corridor_x1 = 0, cell_corridor_x2 = 0;

    // cout << "ESTOU DENTRO DO PERSON_COUNT" << endl;
    // for (int x = cell_x - 3; x < cell_x + 3; x++) {
    //     for (int y = cell_y - corridor_threshold; y < cell_y + corridor_threshold; y++) {
        
        
    //         if ((y >= 0 && y < map_height) && (x >= 0 && x < map_width)) {
    //             map_custom_[x][y] = 100;
    //         }
    //     }
        // if (map_custom_[x][cell_y] == 100 && !first_cell && !blank_cell) {
        //     first_cell = true;lane_map_update_ FIRST CELL: " << matrix2vectorIndex(x,cell_y,4000) << endl;
        //     cell_corridor_x1 = x-1;
        //     blank_cell = true; 2451 2793
        // }
    // }
    // if (cell_corridor_x1 == 0 || cell_corridor_x2 == 0) {
    //     cout << "ALGUNS DOS IFS NAO PASSOU | x1: " << cell_corridor_x1 << " | x2: " << cell_corridor_x2 << endl;
    // }

    for (int y = cell_y - corridor_threshold; y < cell_y + corridor_threshold; y++) {
        // map_custom_[cell_x][y] = 180;
        if (map_custom_[cell_x][y] == 100 && !first_cell && !blank_cell) {
            cell_corridor_x1 = y;
            first_cell = true;
        }

        if (map_custom_[cell_x][y] == 0 && !blank_cell) {
            if ((map_custom_[cell_x][y-1] != 0) && (map_custom_[cell_x][y+1] == 0)) {
                blank_cell = true;
            }
        }

        if (map_custom_[cell_x][y] == 100 && first_cell && blank_cell) {
            cell_corridor_x2 = y;
            break;
        }

    }

    cout << "CELL_CORRIDOR_X1: " << cell_corridor_x1 << " | CELL_CORRIDOR_X2: " << cell_corridor_x2 << endl;
    if (cell_corridor_x1 != 0 && cell_corridor_x2 == 0) {
        cell_corridor_x2 = cell_corridor_x1 + corridor_threshold;
        
        if (cell_corridor_x1 < cell_corridor_x2) {
            for (int x = cell_corridor_x1; x < cell_corridor_x2; x++) {
                map_custom_[cell_x][x] = 100;
            }
        } else {
            for (int x = cell_corridor_x2; x < cell_corridor_x1; x++) {
                map_custom_[cell_x][x] = 100;
            }
        }
    } else if (cell_corridor_x1 == 0 && cell_corridor_x2 != 0) {
        cell_corridor_x1 = cell_corridor_x2 - corridor_threshold;
        
        if (cell_corridor_x1 < cell_corridor_x2) {
            for (int x = cell_corridor_x1; x < cell_corridor_x2; x++) {
                map_custom_[cell_x][x] = 100;
            }
        } else {
            for (int x = cell_corridor_x2; x < cell_corridor_x1; x++) {
                map_custom_[cell_x][x] = 100;
            }
        }
    } else if (cell_corridor_x1 != 0 && cell_corridor_x2 != 0) {
        if (cell_corridor_x1 < cell_corridor_x2) {
            for (int x = cell_corridor_x1; x < cell_corridor_x2; x++) {
                map_custom_[cell_x][x] = 100;
            }
        } else {
            for (int x = cell_corridor_x2; x < cell_corridor_x1; x++) {
                map_custom_[cell_x][x] = 100;
            }
        }
    }

    // if (cell_corridor_x1 == 0) {
    //     cell_corridor_x1 = corridor_threshold;
    // } else {
    //     if (cell_corridor_x2 == 0) {
    //         cell_corridor_x2 = cell_corridor_x1 + corridor_threshold;
    //     }
    // }
    cout << "CELL_CORRIDOR_X1: " << cell_corridor_x1 << " | CELL_CORRIDOR_X2: " << cell_corridor_x2 << endl;
    already_mark_crowd_ = true;
    // if (cell_corridor_x1 < cell_corridor_x2) {
    //     for (int x = cell_corridor_x1; x < cell_corridor_x2; x++) {
    //         map_custom_[cell_x][x] = 100;
    //     }
    // } else {
    //     for (int x = cell_corridor_x2; x < cell_corridor_x1; x++) {
    //         map_custom_[cell_x][x] = 100;
    //     }
    // }
    
}

void map_update(float pos_x, float pos_y, int cell_value){
    int width, heigth;
    
    width = grid_map_.info.width;
    heigth = grid_map_.info.height;

    //Comentado temporariamente, descomentar depois de fazer os testes
    // write_object_in_file(cell_value,pos_x,pos_y);

    int cell_x, cell_y;

    tie(cell_x, cell_y) = odom2cell(pos_x, pos_y);
    int index;

    int radius = 3;
    for (int y = cell_y - radius; y < cell_y + radius; y++) {
        for (int x = cell_x - radius; x < cell_x + radius; x++) {
            if ((x >= 0 && x < map_width) && (y >= 0 && y < map_height)) {
                map_[x][y] = cell_value;
            }
        }
    }

    
        // VOU ADICIONAR O PERSON_COUNT() AQUI, POIS O INDICE DA CELULA
        // JÁ ESTÁ DISPONÍVEL NESTA FUNÇÃO;
        if (is_crowd_) {
            if (cell_value == PERSON_VALUE) {
                // cout << "ENTREI, VOU CHAMAR O PERSON_COUNT" << endl;
                // cout << "CELL_X: " << cell_x << " | CELL_Y: " << cell_y << " | CELL_VALUE: " << cell_value << endl;
                //MUDAR O NOME DESSA FUNCAO DEPOIS
                person_count(cell_x, cell_y);
            }
        }
        
    

    can_publish = true;
}

//----------------------------------------------------------------
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
            if (map_[x][y] == SUITCASE_VALUE || map_[x][y] == PERSON_VALUE || map_[x][y] == BICYCLE_VALUE || map_[x][y] == VASE_VALUE) {
                map_[x][y] = 0;
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
            
            if (map_[x][y] == SUITCASE_VALUE || map_[x][y] == PERSON_VALUE || map_[x][y] == BICYCLE_VALUE || map_[x][y] == VASE_VALUE) {
                map_[x][y] = 0;
            }      
        }
    }    
}

void scanForObejctsInMap(){
    int cell_x, cell_y;
    int radius_y = 150;
    int radius_x = 1;
    int radius_cell = 300;
    int odom_y,odom_x;
    float laser_x, laser_y,robot_rad;
    float beta = HFOV_RAD/2;

    tie(cell_x, cell_y) = odom2cell(camera_pose[0], camera_pose[1]);

    for (int ang = 0; ang < MAX_BEAMS; ang++) {
        robot_rad = remainder(ROBOT_POSE_[2]+((ang*ANGLE_INCREASE)-beta),2.0*M_PI);
        laser_x = (cos(robot_rad) * MAX_RANGE_CAM_DEPTH);
        laser_y = (sin(robot_rad) * MAX_RANGE_CAM_DEPTH);
        tie(odom_x,odom_y) = odom2cell(camera_pose[0]+laser_x,camera_pose[1]+laser_y);
        if ((odom_x >= 0 && odom_x < map_width) && (odom_y >= 0 && odom_y < map_height)) {
            himm_inc(cell_x,cell_y,odom_x,odom_y);
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

    float object_x, object_y;

    std::tie(object_x,object_y) = cloudTransform(object_pos_x,object_pos_y);

    //--------------------------ADICIONADO 14/12-----------------------------------------------
    //--------------------------NECESSARIO QUE SEJA REALIZADO TESTE DE FUNCIONAMENTO-----------
    if (cell_value == PERSON_VALUE) {
        tuple<int,int> pos;    
        int cell_x = 0, cell_y = 0;
        tie(cell_x, cell_y) = odom2cell(object_x, object_y);
        pos = make_tuple(cell_x,cell_y);
        if (p_cont_aux_ == 0) {
            p_cont_aux_++;
            //salva posicao
            person_barrier_.emplace_back(pos);
        } else if (p_cont_aux_ == p_cont_-1) {
            //salva posicao
            person_barrier_.emplace_back(pos);
        } else {
            p_cont_aux_++;
        }
    }
    //-----------------------------------------------------------------------------------------

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
    box_class.clear();
    for (int x = 0; x < msg->bounding_boxes.size(); x++){
        vector<int> t;
        t.push_back(msg->bounding_boxes[x].xmin);
        t.push_back(msg->bounding_boxes[x].xmax);
        t.push_back(msg->bounding_boxes[x].ymin);
        t.push_back(msg->bounding_boxes[x].ymax);
        box_teste.push_back(t);
        box_class.push_back(msg->bounding_boxes[x].Class);
    }
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
                            cout << "CLASS: " << box_class[x] << endl;
                            if (box_class[x] == "suitcase") {            
                                check_object_position(meanDepth,object_pos_x,object_pos_y,SUITCASE_VALUE);
                            } else if (box_class[x] == "person") {                                
                                check_object_position(meanDepth,object_pos_x,object_pos_y,PERSON_VALUE);
                            } else if (box_class[x] == "vase") {
                                check_object_position(meanDepth,object_pos_x,object_pos_y,VASE_VALUE);
                            } else if (box_class[x] == "bicycle") {
                                check_object_position(meanDepth,object_pos_x,object_pos_y,BICYCLE_VALUE);
                            } else {
                                cout << "UNKNOW CLASS, ADD TO MAP WITH UNKWON VALUE JUST TO CHECK LATER" << endl;
                                check_object_position(meanDepth,object_pos_x,object_pos_y,UNKWON_VALUE);
                            }
                        }
                    }
                }
            }
        } 
    }
    all_objects_marked_ = true;
}

void person_verification(){
    // cout << "QUANTIDADE DE OBJETOS A SER ANALISADOS: " << box_teste.size() << endl;
    /*
    TESTE PARA VER SE TEM VÁRIAS PESSOAS NO LOCAL QUE O ROBÔ ESTA VERIFICANDO
    */
    // cout << "REALIZANDO A VERIFICACAO DA QUANTIDADE DE PESSOAS" << endl;
    int p_count = 0;
    for (int x = 0; x < box_teste.size(); x++) {
        if (box_class[x] == "person") {
            p_count++;
        }
    }
    // cout << "QUANTIDADE DE PESSOAS QUE FORAM ACHADAS: " << p_count << endl; 

    p_cont_ = p_count;

    if (p_count > 2) {
        is_crowd_ = true;
        if (!crowd_published_) {
            already_mark_crowd_ = false;
        }
    } else {
        is_crowd_ = false;
        already_mark_crowd_ = false;
        crowd_published_ = false;
    }
    //******************************************************************************************************
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
        if (!can_publish){
            for (int x = 0; x < map_msg->info.width*map_msg->info.height; x++) {
                if (grid_map_.data[x] != SUITCASE_VALUE || grid_map_.data[x] != PERSON_VALUE || grid_map_.data[x] != VASE_VALUE || grid_map_.data[x] != BICYCLE_VALUE) {
                    grid_map_.data[x] = map_msg->data[x];
                }
            }
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

    for (size_t i = 0; i < depthPoints; ++i, ++iter_x, ++iter_y, ++iter_z) {
        depthCloud_[i].x = *iter_x;
        depthCloud_[i].y = *iter_y;
        depthCloud_[i].z = *iter_z;
    }
}

void found_object_callback(const darknet_ros_msgs::ObjectCount& count_msg){
    count_objects_ = count_msg.count;
}

//----------------------------------------------------------------------------------------------------------
void finished_mission_callback(const std_msgs::Bool& finished_mission_msg){
    mission_finished_ = finished_mission_msg.data;
}
//----------------------------------------------------------------------------------------------------------

void lane_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg){
    lane_map_.header.frame_id = map_msg->header.frame_id;
    lane_map_.header.seq = map_msg->header.seq;
    lane_map_.header.stamp = map_msg->header.stamp;
    lane_map_.info.resolution = map_msg->info.resolution;
    lane_map_.info.origin = map_msg->info.origin;
    lane_map_.info.height = map_msg->info.height;
    lane_map_.info.width = map_msg->info.width;
    if (setup_lane_map_) {
        lane_map_.data = map_msg->data;
        copy_map_custom();
        setup_lane_map_ = false;
    } else {
        if (!can_publish){
            for (int x = 0; x < map_msg->info.width*map_msg->info.height; x++) {
                lane_map_.data[x] = map_msg->data[x];
            }
        }
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "semantic");
    ros::NodeHandle node;

    init_map();
    init_map_custom();

    ros::Subscriber odom_sub = node.subscribe("/husky1/odometry/filtered", 1000, robot_pos_callback);
    ros::Subscriber dark_sub = node.subscribe("/darknet_ros/bounding_boxes", 1000, darknet_callback);
    ros::Subscriber grid_map = node.subscribe("/map_out", 1, grid_callback);
    ros::Subscriber lane_map_sub = node.subscribe("/lane_map",1,lane_map_callback);
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

    //ADICIONADO 24/11/22 | 25/11/22
    //TESTE DE BLOQUEIO NO MAPA
    ros::Publisher lane_map_pub = node.advertise<nav_msgs::OccupancyGrid>("/lane_map",10);
    
    //-------------------------------------------------------------------------------

    ros::Rate rate(10);
    
    while(ros::ok()){
        basefootprintToCameraTF();
        if (cv_ptr_){
            // checkGridForValue();
            person_verification();
            if (is_crowd_ && !already_mark_crowd_) {
                copy_map_custom();
                boundToSemanticMap();
                if (can_publish) {
                    grid_update_custom();
                    lane_map_pub.publish(lane_map_);
                    can_publish = false;
                    crowd_published_ = true;
                }
            }
            if (mission_finished_){    
                if (count_objects_ > 0) {
                    copy_map();
                    // copy_map_custom();
                    boundToSemanticMap();
                    if (can_publish) {
                        grid_update();                        
                        map_pub.publish(grid_map_);
                        /*
                            ********************************ADICIONADO 24/11/22********************************
                            REALIZANDO A PUBLICAÇÃO DOS BLOQUEIOS DENTRO DO LANE_MAP
                        */
                        // grid_update_custom();
                        // lane_map_pub.publish(lane_map_);
                        //**************************************************************************************************

                        can_publish = false;
                        //**************************************************************************************************
                        //FOR TEST PURPOSES, NEED TO BE EXCLUDED LATER
                        img_out_->header.frame_id = cv_ptr_->header.frame_id;
                        img_out_->header.seq = cv_ptr_->header.seq;
                        img_out_->header.stamp = cv_ptr_->header.stamp;
                        img_out_->encoding = cv_ptr_->encoding;
                        img_out.publish(img_out_->toImageMsg());
                        //**************************************************************************************************
                        ros_finished_marking_.data = true;
                        finished_marking_pub.publish(ros_finished_marking_);
                    } else {
                        ros_finished_marking_.data = false;
                        finished_marking_pub.publish(ros_finished_marking_);
                    }
                } else {
                    copy_map();
                    copy_map_custom();
                    scanForObejctsInMap();
                    grid_update(); 
                    grid_update_custom();  
                    map_pub.publish(grid_map_);
                    lane_map_pub.publish(lane_map_);
                    ros_finished_marking_.data = true;
                    finished_marking_pub.publish(ros_finished_marking_);
                }
            } else {
                ros_finished_marking_.data = false;
                finished_marking_pub.publish(ros_finished_marking_);
                all_objects_marked_ = false;
            }
            
        }
            ros::spinOnce();
            rate.sleep();
    }

    return 0;
}