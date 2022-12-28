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
#include <algorithm>

using namespace std;
using namespace Eigen;

typedef struct {
    float x,y,z;
} Point3D;

nav_msgs::OccupancyGrid grid_map_;
nav_msgs::OccupancyGrid lane_map_;
nav_msgs::OccupancyGrid crowd_layer_map_;
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

float ROBOT_POSE_[3] = {0,0,0};
float AMCL_POSE_[3] = {0,0,0};
const float ANGLE_LIMITS_MARK_[4] = {1.10,2.35,-2.0,-0.5};
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
int crowd_temporal_layer_map_[4000][4000];
vector<tuple<int,int,bool>> vertex_;
vector<tuple<int,int,std::chrono::system_clock::time_point,int,bool,int>> ctl_tau_;
bool both_zero = false;
bool is_edge = false;
vector<tuple<int,int>> person_pos_;
vector<float> person_pos_x_aux_;
vector<float> person_pos_y_aux_;
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

//---------------------------------ADICIONADO 15/12/22-------------------------------------------
/*
ESTAS FUNCOES FORAM COMPILADAS E TESTADAS, CONTUDO É NECESSÁRIO REALIZAR A LIMPEZA DOS
CÓDIGOS E MODIFICAR OS NOMES DE AMBAS. AINDA É NECESSÁRIO VERIFICAR UMA FORMA DE FAZER
ESTAS FUNÇÕES DE FORMA GENÉRICA, POR HORA AINDA MANTENHO ELAS ESPECIFICAS PARA CADA MAPA
QUE É CRIADO NO SISTEMA.
*/
void init_crowd_temporal_map_custom(){
    for (int x = 0; x < 4000; x++) {
        for (int y = 0; y < 4000; y++) {
            crowd_temporal_layer_map_[x][y] = -1;
        }
    }
    crowd_layer_map_.info.height = 4000;
    crowd_layer_map_.info.width = 4000;
    crowd_layer_map_.info.resolution = 0.05;
    crowd_layer_map_.info.origin.position.x = map_origin_x;
    crowd_layer_map_.info.origin.position.y = map_origin_y;
}
void grid_update_crowd_temporal_custom(){
    for(int x = 0; x < map_width; x++){
        int multi = x * map_width;
        for(int y = 0; y < map_height; y++){
            int index = y + multi;
            crowd_layer_map_.data[index] = crowd_temporal_layer_map_[x][y];
        }
    }
}
//-------------------------------------------------------------------------------------------------

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

    //Colocar o valor das células armazenadas dentro das variáveis cell_
    // tie(cell_x,cell_y) = person_barrier_[0];
    cout << "QNT QUE A LINHA ACIMA FOI EXECUTADA: " << p_cont_aux_ << endl;
    // person_barrier_.erase(person_barrier_.begin());

    cout << "AMCL_ANGLE_POSE: " << AMCL_POSE_[2] << endl;
    

    if ((AMCL_POSE_[2] > ANGLE_LIMITS_MARK_[0] && AMCL_POSE_[2] <= ANGLE_LIMITS_MARK_[1]) || (AMCL_POSE_[2] > ANGLE_LIMITS_MARK_[2] && AMCL_POSE_[2] <= ANGLE_LIMITS_MARK_[3])) {
        // map_custom_[cell_x - corridor_threshold][cell_y] = 125;
        cout << "&&&&&&&&&&&&&&&&&&&&&&&&SO PRA SABER QUE ESTOU AQUI" << endl;
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
                both_zero = false;
                break;
            }

        }
    } else if ((AMCL_POSE_[2] > ANGLE_LIMITS_MARK_[1] || AMCL_POSE_[2] <= ANGLE_LIMITS_MARK_[2]) || (AMCL_POSE_[2] < ANGLE_LIMITS_MARK_[0] && AMCL_POSE_[2] >= ANGLE_LIMITS_MARK_[3])) {
        // map_custom_[cell_x - corridor_threshold][cell_y] = 140;
        cout << "@@@@@@@@@@@@@@@@@@@VALOR DO CELL: [ " << cell_x << " | " << cell_x - corridor_threshold << " ]" << endl; 
        for (int y = cell_x - corridor_threshold; y < cell_x + corridor_threshold; y++) {
            // map_custom_[y][cell_y] = 220;
            if (map_custom_[y][cell_y] == 100 && !first_cell && !blank_cell) {
                cell_corridor_x1 = y;
                first_cell = true;
                // map_custom_[y][cell_y] = 180;
            }

            if (map_custom_[y][cell_y] == 0 && !blank_cell) {
                if ((map_custom_[y+1][cell_y] != 0) && (map_custom_[y-1][cell_y] == 0)) {
                    blank_cell = true;
                    // map_custom_[y][cell_y] = 150;
                }
            }

            if (map_custom_[y][cell_y] == 100 && first_cell && blank_cell) {
                cell_corridor_x2 = y;
                both_zero = true;
                // map_custom_[y][cell_y] = 110;
                break;
            }

            // map_custom_[y][cell_y] = 150;

        }
    }

    // for (int y = cell_y - corridor_threshold; y < cell_y + corridor_threshold; y++) {
    //     // map_custom_[cell_x][y] = 180;
    //     if (map_custom_[cell_x][y] == 100 && !first_cell && !blank_cell) {
    //         cell_corridor_x1 = y;
    //         first_cell = true;
    //     }

    //     if (map_custom_[cell_x][y] == 0 && !blank_cell) {
    //         if ((map_custom_[cell_x][y-1] != 0) && (map_custom_[cell_x][y+1] == 0)) {
    //             blank_cell = true;
    //         }
    //     }

    //     if (map_custom_[cell_x][y] == 100 && first_cell && blank_cell) {
    //         cell_corridor_x2 = y;
    //         both_zero = false;
    //         break;
    //     }

    // }

    
    cout << "!@@!@!@!@!@!@!@!@!@!@!@!@ | CELL_CORRIDOR_X1: " << cell_corridor_x1 << " | CELL_CORRIDOR_X2: " << cell_corridor_x2 << " !@!@!@!@@!@!@!@!@" << endl;
    if (cell_corridor_x1 == 0 && cell_corridor_x2 == 0) {
        cout << "@*!&@*!&@*!&@*!&@*!&@*&!@ NAO ERA PRA ENTRAR AQUI" << endl;
        first_cell = false;
        blank_cell = false;
        // map_custom_[cell_x - corridor_threshold][cell_y] = 140;
        for (int y = cell_x - corridor_threshold; y < cell_x + corridor_threshold; y++) {
            // map_custom_[y][cell_y] = 220;
            if (map_custom_[y][cell_y] == 100 && !first_cell && !blank_cell) {
                cell_corridor_x1 = y;
                first_cell = true;
            }

            if (map_custom_[y][cell_y] == 0 && !blank_cell) {
                if ((map_custom_[y-1][cell_y] != 0) && (map_custom_[y+1][cell_y] == 0)) {
                    blank_cell = true;
                }
            }

            if (map_custom_[y][cell_y] == 100 && first_cell && blank_cell) {
                cell_corridor_x2 = y;
                both_zero = true;
                break;
            }

        }
    } else {
        both_zero = false;
    }

    cout << "AQUIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII | CELL_CORRIDOR_X1: " << cell_corridor_x1 << " | CELL_CORRIDOR_X2: " << cell_corridor_x2 << endl;
    if ((AMCL_POSE_[2] > ANGLE_LIMITS_MARK_[1] || AMCL_POSE_[2] <= ANGLE_LIMITS_MARK_[2]) || (AMCL_POSE_[2] < ANGLE_LIMITS_MARK_[0] && AMCL_POSE_[2] >= ANGLE_LIMITS_MARK_[3])) {
        cout << "####################AMBOS OS CEEL_CORRIDORS ERAM ZERO!!!!!!!!!!!!!!!!!!" << endl;
        if (cell_corridor_x1 != 0 && cell_corridor_x2 == 0) {
            blank_cell = false;
            for (int y = cell_corridor_x1 + 1; y < cell_corridor_x1 + (2*corridor_threshold); y++) {
                if (map_custom_[y][cell_y] == 0 && !blank_cell) {
                    if ((map_custom_[y-1][cell_y] != 0) && (map_custom_[y+1][cell_y] == 0)) {
                        blank_cell = true;
                    }
                }

                if (map_custom_[y][cell_y] == 100 && blank_cell) {
                    cell_corridor_x2 = y;
                    both_zero = true;
                    break;
                }
            }

            if (cell_corridor_x2 == 0) {
                cell_corridor_x2 = cell_corridor_x1 - corridor_threshold;
            }

            // cell_corridor_x2 = cell_corridor_x1 - corridor_threshold;
            
            if (cell_corridor_x1 < cell_corridor_x2) {
                for (int x = cell_corridor_x1; x < cell_corridor_x2; x++) {
                    map_custom_[x][cell_y] = 100;
                }
            } else {
                // map_custom_[cell_corridor_x2][cell_y] = 120;
                for (int x = cell_corridor_x2; x < cell_corridor_x1; x++) {
                    map_custom_[x][cell_y] = 100;
                }
            }
        } else if (cell_corridor_x1 == 0 && cell_corridor_x2 != 0) {
            cell_corridor_x1 = cell_corridor_x2 + corridor_threshold;
            
            if (cell_corridor_x1 < cell_corridor_x2) {
                for (int x = cell_corridor_x1; x < cell_corridor_x2; x++) {
                    map_custom_[x][cell_y] = 100;
                }
            } else {
                for (int x = cell_corridor_x2; x < cell_corridor_x1; x++) {
                    map_custom_[x][cell_y] = 100;
                }
            }
        } else if (cell_corridor_x1 != 0 && cell_corridor_x2 != 0) {
            if (cell_corridor_x1 < cell_corridor_x2) {
                for (int x = cell_corridor_x1; x < cell_corridor_x2; x++) {
                    map_custom_[x][cell_y] = 100;
                }
            } else {
                for (int x = cell_corridor_x2; x < cell_corridor_x1; x++) {
                    map_custom_[x][cell_y] = 100;
                }
            }
        }
    } else if ((AMCL_POSE_[2] > ANGLE_LIMITS_MARK_[0] && AMCL_POSE_[2] <= ANGLE_LIMITS_MARK_[1]) || (AMCL_POSE_[2] > ANGLE_LIMITS_MARK_[2] && AMCL_POSE_[2] <= ANGLE_LIMITS_MARK_[3])) {
        cout << "***********SO UM DOS CEEL_CORRIDORS ERAM ZERO!!!!!!!!!!!!!!!!!!" << endl;
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
    }
    

    /*
        CELL_X = POSICAO Y DO MAPA;
        CELL_CORRIDOR_X1 E 2 = POSICAO X DO MAPA;
        COMBINANDO PODE-SE OBTER O VERTICE1 E 2 E 3. GERANDO COMO CONSEQUENCIA O VERTICE 4;
        (cell_corridor_x1,cell_x);(cell_corridor_x2,cell_x)        
    */
    tuple<int,int,bool> corridor_aux;
    if ((AMCL_POSE_[2] > ANGLE_LIMITS_MARK_[1] || AMCL_POSE_[2] <= ANGLE_LIMITS_MARK_[2]) || (AMCL_POSE_[2] < ANGLE_LIMITS_MARK_[0] && AMCL_POSE_[2] >= ANGLE_LIMITS_MARK_[3])) {
        // if (p_cont_aux_ == 1 || p_cont_aux_ == 0) {
        cout << "SALVANDO OS VERTEX!!!!!!!!!        CELL_y: " << cell_y << "CELL_C1|2: [" << cell_corridor_x1 << " ; " << cell_corridor_x2 << " ]" << endl;
            corridor_aux = make_tuple(cell_y,cell_corridor_x1,both_zero);
            vertex_.emplace_back(corridor_aux);
            corridor_aux = make_tuple(cell_y,cell_corridor_x2,both_zero);
            vertex_.emplace_back(corridor_aux);
        // }
    } else {
        // if (p_cont_aux_ == 1 || p_cont_aux_ == 0) {
        cout << "SALVANDO OS VERTEX!!!!!!!!!        CELL_X: " << cell_x << "CELL_C1|2: [" << cell_corridor_x1 << " ; " << cell_corridor_x2 << " ]" << endl;
            corridor_aux = make_tuple(cell_corridor_x1,cell_x,both_zero);
            vertex_.emplace_back(corridor_aux);
            corridor_aux = make_tuple(cell_corridor_x2,cell_x,both_zero);
            vertex_.emplace_back(corridor_aux);
        // }
    }
    

    // if (cell_corridor_x1 == 0) {
    //     cell_corridor_x1 = corridor_threshold;
    // } else {
    //     if (cell_corridor_x2 == 0) {
    //         cell_corridor_x2 = cell_corridor_x1 + corridor_threshold;
    //     }
    // }
    cout << "CELL_CORRIDOR_X1: " << cell_corridor_x1 << " | CELL_CORRIDOR_X2: " << cell_corridor_x2 << " | CELL_X: " << cell_x << endl;
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

    // is_edge = false;
    
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
                if (is_edge) {
                    for (int x = 0; x < person_pos_.size(); x++) {
                        int cx,cy;
                        tie(cx,cy) = person_pos_[x];
                        person_count(cx, cy);
                    }
                    is_edge = false;
                    // person_count(cell_x, cell_y);
                }
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
    //     tuple<int,int> pos;    
    //     int cell_x = 0, cell_y = 0;
    //     tie(cell_x, cell_y) = odom2cell(object_x, object_y);
    //     pos = make_tuple(cell_x,cell_y);
    //     float e_diff_p1, e_diff_p2;
    //     e_diff_p1 = (AMCL_POSE_[0] - object_x);
    //     e_diff_p2 = (AMCL_POSE_[1] - object_y);
    //     float eucli_diff = sqrt((pow(e_diff_p1,2)+pow(e_diff_p2,2)));
        cout << "POS_ROBOT : [ " << AMCL_POSE_[0] << " ; " << AMCL_POSE_[1] << " ]" << endl; 
        cout << "POS_ODOM : [ " << object_x << " ; " << object_y << " ]" << endl;
    //     cout << "E_DIFF_P1: " << e_diff_p1 << " | E_DIFF_P2: " << e_diff_p2 << endl;
    //     cout << "EUCLIDEAN DIFF BETWEEN ROBOT AND OBJECT: " << eucli_diff << endl;
    //     if (eucli_diff < 7.5) {
    //         if (p_cont_aux_ == 0) {
    //             p_cont_aux_++;
    //             //salva posicao
    //             // person_barrier_.emplace_back(pos);
                
    //                 person_barrier_.emplace(person_barrier_.begin(),pos);
    //                 cout << "POS_CELL PERSON 1: [ " << cell_x << " ; " << cell_y << " ]" << endl; 
    //                 cout << "POS_ODOM PERSON 1: [ " << object_x << " ; " << object_y << " ]" << endl;
    //                 is_edge = true;
                
                
    //         } else if (p_cont_aux_ == p_cont_-1) {
    //             //salva posicao
    //             // person_barrier_.emplace_back(pos);
    //             // if (eucli_diff < 5) {
    //                 person_barrier_.emplace(person_barrier_.begin(),pos);
    //                 cout << "POS_CELL PERSON 2: [ " << cell_x << " ; " << cell_y << " ]" << endl;
    //                 cout << "POS_ODOM PERSON 2: [ " << object_x << " ; " << object_y << " ]" << endl;
    //                 // p_cont_aux_ = 0;
    //                 is_edge = true;
    //             // }
    //         } else {
    //             p_cont_aux_++;
    //         }
    //     }
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

//--------------------------ADICIONADO 19/12/22--------------------------------------------
int limit_verification(int alpha){
    int x,y,cont,alpha_aux,gama = 5;
    bool to_analyze,is_used=false;
    std::chrono::system_clock::time_point tempo;
    
    int maior = 0;

    for (int i = 0; i < ctl_tau_.size(); i++) {
        for (int j = 0; j < ctl_tau_.size(); j++) {
            tie(x,y,tempo,cont,to_analyze,alpha_aux) = ctl_tau_[i];
            if (alpha_aux > maior) {
                maior = alpha_aux;
            }    
        }
    }

    for (int i = 0; i < ctl_tau_.size(); i++) {
        tie(x,y,tempo,cont,to_analyze,alpha_aux) = ctl_tau_[i];
        if (alpha == alpha_aux) {
            is_used = true;
            break;
        }
    }

    if (!is_used) {
        return alpha;
    } else {
        if ((alpha+gama) > 240) {
            alpha = 240;
            return alpha;
        } else {
            return (alpha+gama);
        }
    }
}
//-----------------------------------------------------------------------------------------

//--------------------------ADICIONADO 15/12/22--------------------------------------------
//--------------------------NECESSARIO QUE SEJA REALIZADO TESTE DE FUNCIONAMENTO-----------
//------------------------CTL = CROWD_TEMPORAL_LAYER---------------------------------------
/*
    ESTA FUNCAO FOI COMPILADA E TESTADA BREVEMENTE. É NECESSÁRIO REALIZAR MAIS TESTES.
    ALÉM DE MODIFICAR O FUNCIONAMENTO DA MARCAÇAO EM FUNÇÃO DO TEMPO E DA QUANTIDADE DE PESSOAS.
*/
void ctl_draw(){
    int vertex1_x,vertex1_y,vertex2_x,vertex2_y,vertex3_x,vertex3_y;
    bool bz1,bz2,bz3;

    tie(vertex1_x,vertex1_y,bz1) = vertex_[0];
    tie(vertex2_x,vertex2_y,bz2) = vertex_[3];
    tie(vertex3_x,vertex3_y,bz3) = vertex_[vertex_.size()-1];

    int aux, alpha, beta=100;

    // cout << "VERTEX2_Y: " << vertex2_y << " | VERTEX3_Y: " << vertex3_y << " | VERTEX1_X: " << vertex1_x << " | VERTEX2_X: " << vertex2_x << endl;  
    cout << "VERTEX_1: [" << vertex1_x << " ; " << vertex1_y << " ] | VERTEX_2: [" << vertex2_x << " ; " << vertex2_y << " ]" << endl; 

    if ((AMCL_POSE_[2] > ANGLE_LIMITS_MARK_[1] || AMCL_POSE_[2] <= ANGLE_LIMITS_MARK_[2]) || (AMCL_POSE_[2] < ANGLE_LIMITS_MARK_[0] && AMCL_POSE_[2] >= ANGLE_LIMITS_MARK_[3])) {
        int lim_11,lim_12,lim_21,lim_22;
        
        if (vertex1_y < vertex2_y) {
            lim_11 = vertex1_y;
            lim_12 = vertex2_y;
        } else {
            lim_12 = vertex1_y;
            lim_11 = vertex2_y;
        }

        if (vertex1_x < vertex2_x) {
            lim_21 = vertex1_x;
            lim_22 = vertex2_x;
        } else {
            lim_22 = vertex1_x;
            lim_21 = vertex2_x;
        }

        cout << "LIM_1: [" << lim_11 << " ; " << lim_12 << " ] | LIM_2: [ " << lim_21 << " ; " << lim_22 << " ]" << endl; 

        for (int y = lim_11; y < lim_12; y++) {
            for (int x = lim_21; x < lim_22; x++) {
                //Posteriormente adicionar cores diferentes que estejam em função do tempo.
                //REALIZAR UM CALCULO PELA QUANTIDADE DE PESSOAS TALVEZ?
                //--------------------------------ADICIONADO 19/12/22---------------------------------
                alpha = (p_cont_ * ctl_tau_.size()) + beta;
                alpha = limit_verification(alpha);
                crowd_temporal_layer_map_[y][x] = alpha;
                //------------------------------------------------------------------------------------
                // cout << "ANTES DO CORE DUMPED!!!!" << endl;
                // crowd_temporal_layer_map_[y][x] = 150;
                // cout << "FOI AQUI QUE DEU CORE DUMPED!!!!" << endl;
            }
        }
    } else {
        int lim_11,lim_12,lim_21,lim_22;
        
        if (vertex1_y < vertex2_y) {
            lim_11 = vertex1_y;
            lim_12 = vertex2_y;
        } else {
            lim_12 = vertex1_y;
            lim_11 = vertex2_y;
        }

        if (vertex1_x < vertex2_x) {
            lim_21 = vertex1_x;
            lim_22 = vertex2_x;
        } else {
            lim_22 = vertex1_x;
            lim_21 = vertex2_x;
        }

        cout << "LIM_1: [" << lim_11 << " ; " << lim_12 << " ] | LIM_2: [ " << lim_21 << " ; " << lim_22 << " ]" << endl; 

        for (int y = lim_11; y < lim_12; y++) {
            for (int x = lim_21; x < lim_22; x++) {
                //Posteriormente adicionar cores diferentes que estejam em função do tempo.
                //REALIZAR UM CALCULO PELA QUANTIDADE DE PESSOAS TALVEZ?
                //--------------------------------ADICIONADO 19/12/22---------------------------------
                alpha = (p_cont_ * ctl_tau_.size()) + beta;
                alpha = limit_verification(alpha);
                crowd_temporal_layer_map_[y][x] = alpha;
                //------------------------------------------------------------------------------------
                // cout << "ANTES DO CORE DUMPED!!!!" << endl;
                // crowd_temporal_layer_map_[y][x] = 150;
                // cout << "FOI AQUI QUE DEU CORE DUMPED!!!!" << endl;
            }
        }
    }
    
    /*
        --> GUARDAR O VERTEX2_Y PARA POSTERIOR ANALISE DO LOCAL;
        --> ATRIBUIR UM TEMPO PARA ESTA MARCACAO NO MAPA;
        --> MANTER INFORMACAO DA QUANTIDADE DE PESSOAS QUE HAVIA NO LOCAL;
    */
    tuple<int,int,std::chrono::system_clock::time_point,int,bool,int> tau;
    tau = make_tuple(vertex2_x/2,vertex2_y,std::chrono::system_clock::now(),p_cont_,true,alpha);
    ctl_tau_.emplace_back(tau);
}
//-----------------------------------------------------------------------------------------

/*
FUNCAO PARA VERIFICAR O TEMPO DECORRIDO DE UMA AREA DO CTL;
ESTA FUNCAO PODE SER MANTIDA TANTO NO HUSKY QUANTO NO PIONEER.
OBS: PARA O PIONEER REALIZAR ESTA FUNCAO, E NECESSARIO QUE O MAP SEJA ENVIADO PARA ELE ATRAVES DOS TOPICOS.
*/
void ctl_time_verification(){
    /*
    --> verificar todas as entradas de ctl do vetor;
    --> fazer a diferença do tempo atual para o tempo da marcacao;
    --> se tempo maior que Tau, obtem a posicao do local;
        --> manda o robo para o local designado;
        --> verifica se o local possui um numero maior ou igual x de pessoas;
            --> caso true: modifica o tempo pelo tempo atual;
            --> caso false: limpa a marcação e limpa a posicao do vetor;

    //--------------------------ADICIONADO 16/12/22---------------------------------
    
        TESTE COM A ATRIBUIÇÃO DE TEMPO EM CADA MARCAÇÃO DO CTL

        COMPILADO E TESTADO! NECESSÁRIO ALGUNS AJUSTES, PARA QUE A POSIÇÃO
        SEJA ENVIADA PARA A FILA DE OBJETIVOS DO ROBÔ DEPOIS QUE O TEMPO MAXIMO
        FOR ATINGIDO PARA CADA MARCAÇÃO.

        AJUSTES NECESSÁRIOS:
            --> AJUSTAR O GOAL PARA O ROBO;
            --> PUBLICAR NA LISTA DE GOALS DO ROBO;
            --> IMPLEMENTAR UM SISTEMA DE PRIORIDADE, POIS O ROBO SÓ VAI FAZER O QUE FOI PREVIAMENTE CARREGADO
                OU DEVE SER ENVIADO PARA O PIONEER, JÁ QUE O PIONEER ESTA COM O SISTEMA PRONTO PARA RECEBER GOALS
                ESPECIFICOS DO SISTEMA VIA TOPICOS.
    */

    int tau_threshold = 120; 
    if (!ctl_tau_.empty()) {
        for (int i = 0; i < ctl_tau_.size(); i++) {
            int x,y,cont,alpha;
            bool to_analyze;
            std::chrono::system_clock::time_point tempo;
            std::chrono::system_clock::time_point tempo_now;
            tempo_now = std::chrono::system_clock::now();
            tie(x,y,tempo,cont,to_analyze,alpha) = ctl_tau_[i];
            std::chrono::duration<double> e_sec = tempo_now - tempo;
            if ((e_sec.count() > tau_threshold) && to_analyze) {
                cout << "******************TESTE DO QUE FOI ARMAZENADO******************" << endl;
                cout << "CELL_X DO CTL: " << x << " | CELL_Y: " << y << " | QNT DE PESSOAS: " << cont << endl;
                cout << "TEMPO SALVO: " << e_sec.count() << endl;
                cout << "***************************************************************" << endl;
                ctl_tau_.at(i) = make_tuple(x,y,tempo,cont,false,alpha);
            }
        }
    }
    //------------------------------------------------------------------------------
}

//-------------------------------------------------------ADICIONADO 27/12/22----------------------------------------
/*
    COPIA DA FUNCAO boundToSemanticMap() para verificar todos os objetos person que tem no ambiente.
    Assim que todos os objetos forem obtidos, verificar qual o objeto mais proximo e o mais distante.

    Copia da função check_object_position() para verificar e armazenar as posições dos objetos pessoas encontrados no ambiente.
*/
void check_object_position_copy(float depth, int object_pos_x, int object_pos_y, int cell_value){

    float object_x, object_y;

    std::tie(object_x,object_y) = cloudTransform(object_pos_x,object_pos_y);

    tuple<int,int> pos;    
    int cell_x = 0, cell_y = 0;
    tie(cell_x, cell_y) = odom2cell(object_x, object_y);
    // pos = make_tuple(cell_x,cell_y);

    // person_pos_.emplace_back(pos);
    // person_pos_x_aux_.emplace_back(cell_x);
    // person_pos_y_aux_.emplace_back(cell_y);

    float e_diff_p1, e_diff_p2;
    e_diff_p1 = (AMCL_POSE_[0] - object_x);
    e_diff_p2 = (AMCL_POSE_[1] - object_y);
    float eucli_diff = sqrt((pow(e_diff_p1,2)+pow(e_diff_p2,2)));
    cout << "POS_ROBOT : [ " << AMCL_POSE_[0] << " ; " << AMCL_POSE_[1] << " ]" << endl; 
    cout << "POS_ODOM : [ " << object_x << " ; " << object_y << " ]" << endl;
    cout << "E_DIFF_P1: " << e_diff_p1 << " | E_DIFF_P2: " << e_diff_p2 << endl;
    cout << "EUCLIDEAN DIFF BETWEEN ROBOT AND OBJECT: " << eucli_diff << endl;
    if (eucli_diff < 13) {
        person_pos_x_aux_.emplace_back(object_x);
        person_pos_y_aux_.emplace_back(object_y);
    }
}

void verifyPersonDistance(){
    
    is_edge = true;

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
                        // objectInSemanticMap(object_pos_x,object_pos_y);
                        // if (!objectInMap_) {
                        if (box_class[x] == "person") {                                
                            check_object_position_copy(meanDepth,object_pos_x,object_pos_y,PERSON_VALUE);
                        }
                        // }
                    }
                }
            }
        } 
    }

    //MUDAR PARA O ODOM, MAIS FACIL DE FAZER O CALCULO!!!!
    //COLOCAR A VERIFICACAO DE QUAL O ANGULO DO ROBO!!! 

    // tuple<int,int> small,large;
    float sx,sy,lx,ly;
    sx = *min_element(person_pos_x_aux_.begin(),person_pos_x_aux_.end());
    lx  = *max_element(person_pos_x_aux_.begin(),person_pos_x_aux_.end());
    // tie(sx,sy) = small;
    // tie(lx,ly) = large;
    cout << "SMALLEST : [ " << sx << " ; " << sy << " ] | LARGEST: [ " << lx << " ; " << ly << " ]" << endl;

    sy = *min_element(person_pos_y_aux_.begin(),person_pos_y_aux_.end());
    ly  = *max_element(person_pos_y_aux_.begin(),person_pos_y_aux_.end());
    // tie(sx,sy) = small;
    // tie(lx,ly) = large;
    // cout << "EM Y - SMALLEST : [ " << sx << " ; " << sy << " ] | LARGEST: [ " << lx << " ; " << ly << " ]" << endl;
    cout << "SMALLEST : [ " << sx << " ; " << sy << " ] | LARGEST: [ " << lx << " ; " << ly << " ]" << endl;


    //------------------------------------SO ESTA OPARTE E NECESSARIA

    //COLOCAR A VERIFICACAO DO ANGULO DO ROBO
    //INVERTER AS VARIAVEIS NAS DUAS SITUACOES
    if ((AMCL_POSE_[2] > ANGLE_LIMITS_MARK_[1] || AMCL_POSE_[2] <= ANGLE_LIMITS_MARK_[2]) || (AMCL_POSE_[2] < ANGLE_LIMITS_MARK_[0] && AMCL_POSE_[2] >= ANGLE_LIMITS_MARK_[3])) {
        auto ity = minmax_element(person_pos_x_aux_.begin(), person_pos_x_aux_.end());
        int min_idy = distance(person_pos_x_aux_.begin(), ity.first);
        int max_idy = distance(person_pos_x_aux_.begin(), ity.second);    
        //DAR EMPLACE NO VETOR COM AMBOS OS VALORES
        int cx,cy;
        tie(cx,cy) = odom2cell(person_pos_x_aux_[min_idy],person_pos_y_aux_[min_idy]);
        cout << "################EM RELACAO A X####################" << endl;
        cout << "MIN QUE EU TENHO: [" << person_pos_x_aux_[min_idy] << " ; " << person_pos_y_aux_[min_idy] << " ]" << endl;
        cout << "MAX QUE EU TENHO: [" << person_pos_x_aux_[max_idy] << " ; " << person_pos_y_aux_[max_idy] << " ]" << endl;
        cout << "################EM RELACAO A X####################" << endl;
        // person_pos_.emplace_back(make_tuple(person_pos_x_aux_[min_idy],person_pos_y_aux_[min_idy]));
        // person_pos_.emplace_back(make_tuple(person_pos_x_aux_[max_idy],person_pos_y_aux_[max_idy]));
        person_pos_.emplace_back(make_tuple(cx,cy));
        tie(cx,cy) = odom2cell(person_pos_x_aux_[max_idy],person_pos_y_aux_[max_idy]);
        person_pos_.emplace_back(make_tuple(cx,cy));
    } else {
        auto itx = minmax_element(person_pos_y_aux_.begin(), person_pos_y_aux_.end());
        int min_idx = distance(person_pos_y_aux_.begin(), itx.first);
        int max_idx = distance(person_pos_y_aux_.begin(), itx.second);
        // person_pos_.emplace_back(make_tuple(person_pos_x_aux_[min_idx],person_pos_y_aux_[min_idx]));
        // person_pos_.emplace_back(make_tuple(person_pos_x_aux_[max_idx],person_pos_y_aux_[max_idx]));
        int cx,cy;
        tie(cx,cy) = odom2cell(person_pos_x_aux_[min_idx],person_pos_y_aux_[min_idx]);
        cout << "################EM RELACAO A Y####################" << endl;
        cout << "MIN QUE EU TENHO: [" << person_pos_x_aux_[min_idx] << " ; " << person_pos_y_aux_[min_idx] << " ]" << endl;
        cout << "MAX QUE EU TENHO: [" << person_pos_x_aux_[max_idx] << " ; " << person_pos_y_aux_[max_idx] << " ]" << endl;
        cout << "################EM RELACAO A Y####################" << endl;
        person_pos_.emplace_back(make_tuple(cx,cy));
        tie(cx,cy) = odom2cell(person_pos_x_aux_[max_idx],person_pos_y_aux_[max_idx]);
        person_pos_.emplace_back(make_tuple(cx,cy));
        //DAR EMPLACE NO VETOR COM AMBOS OS VALORES
    }    
    //------------------------------------SO ESTA OPARTE E NECESSARIA

    // cout << "TESTE COM O ITERATOR: MAX: " << max_idx << " | MIN: " << min_idx << endl;
    // cout << "ELEMNETOS: MAX: " << person_pos_x_aux_[max_idx] << " | MIN: " << person_pos_x_aux_[min_idx] << endl;
}
//------------------------------------------------------------------------------------------------------------------

void boundToSemanticMap(){
    
    verifyPersonDistance();
    
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
    //--------------------------ADICIONADO 15/12/22--------------------------------------------
    //--------------------------NECESSARIO QUE SEJA REALIZADO TESTE DE FUNCIONAMENTO-----------
    ctl_draw();
    vertex_.clear();
    person_barrier_.clear();
    person_pos_.clear();
    person_pos_x_aux_.clear();
    person_pos_y_aux_.clear();
    //-----------------------------------------------------------------------------------------
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
            //------------------------ADICIONADO 27/12/22--------------------------------
            // tuple<int,int> pos;    
            // int cell_x = 0, cell_y = 0;
            // tie(cell_x, cell_y) = odom2cell(object_x, object_y);
            // pos = make_tuple(cell_x,cell_y); 
            // person_pos_.emplace_back(pos);       
            //---------------------------------------------------------------------------
        }
    }
    // cout << "QUANTIDADE DE PESSOAS QUE FORAM ACHADAS: " << p_count << endl; 

    p_cont_ = p_count;

    if (p_count > 2) {
        is_crowd_ = true;
        if (!crowd_published_) {
            already_mark_crowd_ = false;
            p_cont_aux_ = 0;
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

//------------------------------------ADICIONADO 19/12/22-----------------------------------
/*
    FOI NECESSÁRIO ADICIONAR AS INFORMAÇÕES DA POSIÇÃO DO AMCL PARA REALIZAR O FECHAMENTO
    DOS CORREDORES EM RELAÇÃO A ORIENTAÇÃO DO ROBÔ.
*/
void amcl_pos_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_msg){
    AMCL_POSE_[0] = amcl_msg->pose.pose.position.x;
    AMCL_POSE_[1] = amcl_msg->pose.pose.position.y;
    Quaternionf q;
    q.x() = amcl_msg->pose.pose.orientation.x;
    q.y() = amcl_msg->pose.pose.orientation.y;
    q.z() = amcl_msg->pose.pose.orientation.z;
    q.w() = amcl_msg->pose.pose.orientation.w;
    auto euler = q.toRotationMatrix().eulerAngles(0,1,2);
    AMCL_POSE_[2] = euler[2];
}
//-----------------------------------------------------------------------------------------

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
        //------------------------ADICIONADO 15/12/22------------------------------
        //PARA QUE O MAPA NÃO ESTEJA VAZIO NO INICIO DO PROGRAMA, COLOQUEI AS INFORMAÇÕES
        //DO LANE_MAP, MAS ESTAS INFORMAÇÕES SÃO ZERADAS ASSIM QUE O CTL_MAP É INICIALIZADO
        crowd_layer_map_.data = map_msg->data;
        //-------
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
    init_crowd_temporal_map_custom();

    ros::Subscriber odom_sub = node.subscribe("/husky1/odometry/filtered", 1000, robot_pos_callback);
    ros::Subscriber dark_sub = node.subscribe("/darknet_ros/bounding_boxes", 1000, darknet_callback);
    ros::Subscriber grid_map = node.subscribe("/map_out", 1, grid_callback);
    ros::Subscriber lane_map_sub = node.subscribe("/lane_map",1,lane_map_callback);
    ros::Subscriber depth_img = node.subscribe("/husky1/realsense/depth/image_rect_raw", 1000, depth_img_callback);
    ros::Subscriber cam_info_sub = node.subscribe("/husky1/realsense/color/camera_info", 1, caminfo_callback);
    ros::Subscriber point_cloud_sub = node.subscribe("/husky1/realsense/depth/color/points", 1, point_cloud_callback);
    ros::Subscriber dkn_object_sub = node.subscribe("/darknet_ros/found_object", 1000, found_object_callback);

    /*
    --------------------------ADICIONADO 19/12/22-----------------------------------------------------------
        ADICIONAR SUBSCRIBER DO AMCL E OBTER A ORIENTAÇÃO EM YAW.
        SERÁ UTILIZADO PARA A VERIFICAÇÃO DE QUAL MEDIDA DEVE SER 
        UTILIZADA PARA REALIZAR O FECHAMENTO DO CORREDOR.
    --------------------------------------------------------------------------------------------------------
    */
    ros::Subscriber amcl_sub = node.subscribe("/husky1/amcl_pose", 1000, amcl_pos_callback);
    //---------------------------------------------------------------------------------------------------------

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

    //ADICIONADO 15/12/22
    //TESTE DO CROWD_TEMPORAL_LAYER
    ros::Publisher crowd_temporal_layer_map_pub = node.advertise<nav_msgs::OccupancyGrid>("/crowd_temporal_layer",10);
    //-------------------------------------------------------------------------------

    ros::Rate rate(10);
    
    while(ros::ok()){
        //------------SOMENTE TESTE, RETIRAR LOGO--------------------
        // cout << "YAW DO ROBO: " << AMCL_POSE_[2] << endl;
        //-----------------------------------------------------------
        basefootprintToCameraTF();
        if (cv_ptr_){
            // checkGridForValue();
            person_verification();
            if (is_crowd_ && !already_mark_crowd_ && mission_finished_) {
                copy_map_custom();
                boundToSemanticMap();
                if (can_publish) {
                    grid_update_custom();
                    lane_map_pub.publish(lane_map_);
                    //---------------------------------------ADICIONADO 15/12/22--------------------------
                    //TESTE DO FUNCIONAMENTO DO CROWD TEMPORAL LAYER
                    // cout << "AAAAAAAANTES" << endl;
                    grid_update_crowd_temporal_custom();
                    crowd_temporal_layer_map_pub.publish(crowd_layer_map_);
                    // cout << "DDDDDDDDDDDDDDDEPOIS" << endl;
                    //-------------------------------------------------------------------------------------
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
            ctl_time_verification();
            
        }
            ros::spinOnce();
            rate.sleep();
    }

    return 0;
}