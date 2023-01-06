#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <std_msgs/Bool.h>
#include <chrono>
#include <sstream>
#include <string>
#include <fstream>

//--------------------------ADICIONADO 03/01/23--------------------------------------
#include "nav_msgs/OccupancyGrid.h"
//-----------------------------------------------------------------------------------

using namespace std::chrono;
using namespace std;

vector<tuple<float,float>> goals;
int index_ = 0;
bool time_started_ = false;
std::chrono::steady_clock::time_point start_time_;
std::chrono::steady_clock::time_point end_time_;
std::ofstream objects_map_file_;
std::ofstream fulllog_file_;
bool first_setup = false;
fstream objects_file_;
vector<tuple<int,float,float>> object_goals_;

//--------------------------ADICIONADO 03/01/23--------------------------------------
nav_msgs::OccupancyGrid grid_map_;
nav_msgs::OccupancyGrid lane_map_;
nav_msgs::OccupancyGrid clean_map_;

bool setup_map = true;
int original_map_[4000][4000];
int modified_map_[4000][4000];

fstream ctldraw_file_;
bool enable_ctldraw_ = false;
bool disable_all_ctldraw_ = false;
bool already_drawed_ = false;
//--------------------------ADICIONADO 06/01/23--------------------------------------
bool enable_patrol_ = true;
//-----------------------------------------------------------------------------------

void mission_goals(){
    tuple<float,float> inv_goals;
    float input_goal_x, input_goal_y;
    // fulllog_file_ << "-----------GOALS--------------" << endl;    

    // inv_goals = make_tuple(16.83527374267578,-4.076648712158203);
    // goals.emplace_back(inv_goals);
    // tie(input_goal_x,input_goal_y) = goals[0];
    // fulllog_file_ << "GOAL 0: [" << input_goal_x << " | " << input_goal_y << "]" << endl;

    // inv_goals = make_tuple(16.635780334472656,2.5355117321014404);
    // goals.emplace_back(inv_goals);
    // tie(input_goal_x,input_goal_y) = goals[1];
    // fulllog_file_ << "GOAL 1: [" << input_goal_x << " | " << input_goal_y << "]" << endl;

    // inv_goals = make_tuple(10.314708709716797,43.58147430419922);
    // goals.emplace_back(inv_goals);
    // tie(input_goal_x,input_goal_y) = goals[2];
    // fulllog_file_ << "GOAL 2: [" << input_goal_x << " | " << input_goal_y << "]" << endl;

    // inv_goals = make_tuple(13.89610767364502,2.775299072265625);
    // goals.emplace_back(inv_goals);
    // tie(input_goal_x,input_goal_y) = goals[3];
    // fulllog_file_ << "GOAL 3: [" << input_goal_x << " | " << input_goal_y << "]" << endl;

    // inv_goals = make_tuple(13.953405380249023,43.73188781738281);
    // goals.emplace_back(inv_goals);
    // tie(input_goal_x,input_goal_y) = goals[4];
    // fulllog_file_ << "GOAL 4: [" << input_goal_x << " | " << input_goal_y << "]" << endl;

    // inv_goals = make_tuple(12.793737411499023,2.642454147338867);
    // goals.emplace_back(inv_goals);
    // tie(input_goal_x,input_goal_y) = goals[5];
    // fulllog_file_ << "GOAL 5: [" << input_goal_x << " | " << input_goal_y << "]" << endl;

    // fulllog_file_ << "-----------GOALS--------------" << endl;

    inv_goals = make_tuple(3.0571606159210205,6.464037895202637);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(7.30134391784668,12.985001564025879);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(1.979628562927246,18.99486541748047);
    goals.emplace_back(inv_goals);
    
    inv_goals = make_tuple(-0.04924583435058594,8.970008850097656);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(10.125747680664062,2.951190948486328);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(10.06579303741455,-7.0252604484558105);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(16.586750030517578,-6.57015323638916);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(25.265361785888672,-11.598235130310059);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(28.645061492919922,-20.9417667388916);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(15.961433410644531,-15.847126007080078);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(11.151702880859375,-6.412162780761719);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(9.609451293945312,0.26978492736816406);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(1.9898185729980469,5.972867488861084);
    goals.emplace_back(inv_goals);

}

void mission_goals_from_file(){
    tuple<float,float> inv_goals;
    float input_goal_x, input_goal_y;
    int cell;
    fulllog_file_ << "-----------GOALS--------------" << endl;    
    for (int x = 0; x < object_goals_.size(); x++) {
        tie(cell,input_goal_x,input_goal_y) = object_goals_[x];
        fulllog_file_ << "GOAL 0: [" << input_goal_x << " | " << input_goal_y << "]" << endl;
        cout << "INDICE: " << x << " CELL: " << cell << " PX: " << input_goal_x << " PY: " << input_goal_y << endl;
        inv_goals = make_tuple(input_goal_x-1.0,input_goal_y-1.0);
        goals.emplace_back(inv_goals);
    }
    fulllog_file_ << "-----------GOALS--------------" << endl;
}

void init_file(std::string arq_name){

    std::stringstream name_stream;
    name_stream << "./sim_time_" << arq_name << ".txt";
    std::string file_name = name_stream.str();
    objects_map_file_.open(file_name,ios::app);
}

void init_fulllog_file(std::string arq_name){

    std::stringstream name_stream;
    name_stream << "./full_log_" << arq_name << ".txt";
    std::string file_name = name_stream.str();
    fulllog_file_.open(file_name,ios::app);
}

void open_file(){
    objects_file_.open("./objects_in_map.txt", ios::in);
    if (objects_file_.is_open()) {
        cout << "FILE OPENED SUCCEFULLY!" << endl;
    } else {
        cout << "COULD NOT OPEN CHOOSEN FILE!" << endl;
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
    objects_file_.close();
}

void write_in_file(int index, int last_index, std::chrono::steady_clock::time_point start_time, std::chrono::steady_clock::time_point end_time, int size){
    if (objects_map_file_.is_open()) {
        if (index < size) {
            std::cout << "GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() << "[s]" << std::endl;
            objects_map_file_ << "GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() << "[s]" << std::endl;
            fulllog_file_ << "GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() << "[s]" << std::endl;
        } else {
            last_index = 0;
            std::cout << "GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() << "[s]" << std::endl;
            objects_map_file_ << "GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() << "[s]" << std::endl;
            fulllog_file_ << "GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() << "[s]" << std::endl;
        }
    }else{
        cout << "POR ALGUM MOTIVO O ARQUIVO NAO PODE SER ABERTO" << endl;
    }
}

//---------------------------------------------------ADICIONADO 03/01/23--------------------------------------------------------------------------------
/*
    Vou adicionar o componente de teste com barreiras dinâmicas obtidas através da análise da quantidade de pessoas no corredor.
    Neste momento estou utilizando as barreiras de maneira fixa, que são adicionadas por meio da leitura do arquivo de barreiras ('ctldraw_teste.txt'),
    que foi feito pelo Husky em uma run separada. Assim que o conceito estiver testado, vou realizar um teste para que seja feito em tempo de execução.

    Configs do Teb para trocar e fazer funcionar estes testes:
        --> dynamic_obstacle_dist = 2.1
        --> weight_dynamic_obstacle = 850.00
        --> weight_dynamic_obstacle_inflation = 8.0
        --> max_global_plan_lookahead_dist = 10.0



    AJUSTAR O SISTEMA DE LOOP E LIMPEZA DO CAMINHO DESENHADO NO LANE_MAP

    A --> B
    160;23.437129974365234;2.652937650680542
    160;20.03060531616211;43.382686614990234
*/

void init_map(){
    for (int x = 0; x < 4000; x++) {
        for (int y = 0; y < 4000; y++) {
            original_map_[x][y] = -1;
            modified_map_[x][y] = -1;
        }
    }
}

void copy_original_map(){
    cout << "ENTREI NO ORIGINAL" << endl;
    for(int x = 0; x < grid_map_.info.width; x++){
        int multi = x * grid_map_.info.width;
        for(int y = 0; y < grid_map_.info.height; y++){
            int index = y + multi;
            original_map_[x][y] = grid_map_.data[index];
        }
    }
}

void copy_modified_map(){
    cout << "ENTREI NO MODIFICADO" << endl;
    for(int x = 0; x < grid_map_.info.width; x++){
        int multi = x * grid_map_.info.width;
        for(int y = 0; y < grid_map_.info.height; y++){
            int index = y + multi;
            modified_map_[x][y] = grid_map_.data[index];
        }
    }
}

void grid_update_custom(){
    for(int x = 0; x < grid_map_.info.width; x++){
        int multi = x * grid_map_.info.width;
        for(int y = 0; y < grid_map_.info.height; y++){
            int index = y + multi;
            lane_map_.data[index] = modified_map_[x][y];
        }
    }
}

void grid_update_clear(){
    for(int x = 0; x < grid_map_.info.width; x++){
        int multi = x * grid_map_.info.width;
        for(int y = 0; y < grid_map_.info.height; y++){
            int index = y + multi;
            clean_map_.data[index] = original_map_[x][y];
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
    // cout << "CHEGUEI AQUI NESTE MOMENTO DEPOIS DO GRID_MAP RECEBER AS INFORMACOES" << endl;

    lane_map_.header.frame_id = map_msg->header.frame_id;
    lane_map_.header.seq = map_msg->header.seq;
    lane_map_.header.stamp = map_msg->header.stamp;
    lane_map_.info.resolution = map_msg->info.resolution;
    lane_map_.info.origin = map_msg->info.origin;
    lane_map_.info.height = map_msg->info.height;
    lane_map_.info.width = map_msg->info.width;
    // cout << "CHEGUEI AQUI NESTE MOMENTO DEPOIS DO LANE_MAP RECEBER AS INFORMACOES" << endl;
    if (setup_map) {
        grid_map_.data = map_msg->data;
        lane_map_.data = map_msg->data;
        copy_original_map();
        copy_modified_map();
        cout << "COPIEI OS MAPAS EM AMBOS OS VETORES!!!!!!!!!!!!" << endl;
        setup_map = false;
    } else {
        // if (!can_publish){
            // for (int x = 0; x < map_msg->info.width*map_msg->info.height; x++) {
                // if (grid_map_.data[x] != SUITCASE_VALUE || grid_map_.data[x] != PERSON_VALUE || grid_map_.data[x] != VASE_VALUE || grid_map_.data[x] != BICYCLE_VALUE) {
                    // lane_map_.data[x] = map_msg->data[x];
                // }
            // }
        // }
    }
}

void enable_ctldraw_obstacles(){
    // --abre o arquivo;--
    ctldraw_file_.open("./ctldraw_teste.txt", ios::in);
    if (ctldraw_file_.is_open()) {
        cout << "FILE OPENED SUCCEFULLY!" << endl;
        
        // --le o arquivo;--
        string line, line_aux;
        int cont = 0;
        int vertexes[4];
        while(getline(ctldraw_file_, line)){
            stringstream st(line);
            while(getline(st, line_aux, ';')){
                cout << line_aux << endl;
                vertexes[cont] = stoi(line_aux);
                cont++;
            }
            cont = 0;
            // --desenha as barreiras;--
            cout << "VERTEXES: [ " << vertexes[0] << " | " << vertexes[1] << " | " << vertexes[2] << " | " << vertexes[3] << " ]" << endl;
            for (int y = vertexes[0]; y < vertexes[1]; y++) {
                for (int x = vertexes[2]; x < vertexes[3]; x++) {
                    modified_map_[y][x] = 100;
                }
            }
            // -------------------------
        }
        already_drawed_ = true;
        // -----------------
    } else {
        cout << "COULD NOT OPEN CHOOSEN FILE!" << endl;
    }
    // -------------------
}

// void disable_all_ctldraw_obstacles(){
    
// }

// void disable_point_ctldraw_obstacle(){
    
// }

void enable_ctldraw_callback(const std_msgs::Bool& ectl_msg){
    enable_ctldraw_ = ectl_msg.data;
}

void disable_all_ctldraw_callback(const std_msgs::Bool& dctl_msg){
    disable_all_ctldraw_ = dctl_msg.data;
}

void enable_patrol_callback(const std_msgs::Bool& epatrol_msg){
    enable_patrol_ = epatrol_msg.data;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------


int main(int argc, char **argv){
    
    ros::init(argc, argv, "SOWDC_move");
    ros::NodeHandle node;
    // std::ofstream sim_time_file;

    //--------------------------ADICIONADO 03/01/23--------------------------------------
    init_map();
    //-----------------------------------------------------------------------------------

    init_file((std::string)argv[2]);
    init_fulllog_file((std::string)argv[2]);
    
    // mission_goals();
    open_file();
    read_file();
    mission_goals_from_file();

    std::stringstream move_base_topic_stream;
    move_base_topic_stream << "/" << (std::string)argv[1] << "/move_base";
    std::string move_base_topic = move_base_topic_stream.str();

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_(move_base_topic);

    move_base_msgs::MoveBaseGoal goals_output;

    //--------------------------ADICIONADO 03/01/23--------------------------------------
    ros::Subscriber grid_map = node.subscribe("/lane_map", 1, grid_callback);
    ros::Publisher lane_map_pub = node.advertise<nav_msgs::OccupancyGrid>("/lane_map",10);
    ros::Publisher map_pub = node.advertise<nav_msgs::OccupancyGrid>("/map",10);

    ros::Subscriber enable_ctldraw = node.subscribe("/enable_ctldraw", 1, enable_ctldraw_callback);
    ros::Subscriber disable_all_ctldraw = node.subscribe("/disable_all_ctldraw", 1, disable_all_ctldraw_callback);
    ros::Subscriber enable_patrol_sub = node.subscribe("/enable_patrol", 1, enable_patrol_callback);
    //-----------------------------------------------------------------------------------

    ros::Rate rate(10);

    float input_goal_x, input_goal_y;
    bool setup = true;
    int last_index = 0;
    int sim_laps = 0;
    bool finished_lap = false;
    bool enable_function = true;

    while(ros::ok()){
        if (grid_map_.info.width > 0) {
        if (!move_base_client_.isServerConnected()){
            ros::spinOnce();
            rate.sleep();
        } else {
            //--------------------------ADICIONADO 03/01/23--------------------------------------
            // cout << "INICIOOOOOOOOOOO" << endl;
            if (enable_function) {
                if (sim_laps < 10) {
                    if (enable_patrol_) {
                        for (int index = 0; index < goals.size(); index++) {
                            tie(input_goal_x,input_goal_y) = goals[index];
                            cout << "GOAL [" << index-1 << " -> " << index << "] FOR " << (std::string)argv[1] << ": [ " << input_goal_x << " | " << input_goal_y << " ] " << endl;
                            fulllog_file_ << "GOAL [" << index-1 << " -> " << index << "] FOR " << (std::string)argv[1] << ": [ " << input_goal_x << " | " << input_goal_y << " ] " << endl;
                            goals_output.target_pose.header.frame_id = "map";
                            goals_output.target_pose.pose.position.x = input_goal_x;
                            goals_output.target_pose.pose.position.y = input_goal_y;
                            goals_output.target_pose.pose.position.z = 0;
                            goals_output.target_pose.pose.orientation.x = 0.0;
                            goals_output.target_pose.pose.orientation.y = 0.0;
                            goals_output.target_pose.pose.orientation.z = 0.25;
                            goals_output.target_pose.pose.orientation.w = 0.95;

                            move_base_client_.sendGoal(goals_output);
                            last_index = index;
                            // index_++;
                            setup = false;
                            if (!time_started_) {
                                time_started_ = true;
                                start_time_ = std::chrono::steady_clock::now();
                            }

                            if (move_base_client_.waitForResult()) {
                                if (!time_started_) {
                                        time_started_ = true;
                                        start_time_ = std::chrono::steady_clock::now();
                                    } else {
                                        end_time_ = std::chrono::steady_clock::now();
                                        // time_started_ = false;
                                        std::cout << "TERMINEI VOU GRAVAR - GOAL [" << last_index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time_ - start_time_).count() << "[s]" << std::endl;
                                        write_in_file(last_index,last_index-1,start_time_,end_time_,goals.size());
                                        start_time_ = std::chrono::steady_clock::now();
                                    }
                                    cout << "LAST_INDEX: " << last_index << " INDEX_: " << index+1 << endl;
                                    fulllog_file_ << "LAST_INDEX: " << last_index << " INDEX_: " << index+1 << endl;
                                    cout << "GOAL [" << last_index << " -> " << index+1 << "] FOR HUSKY: [ " << input_goal_x << " | " << input_goal_y << " ] " << endl;
                                    fulllog_file_ << "GOAL [" << last_index << " -> " << index+1 << "] FOR HUSKY: [ " << input_goal_x << " | " << input_goal_y << " ] " << endl;
                            }
                        }
                        if (sim_laps % 2 != 0) {
                            clean_map_ = grid_map_;
                            grid_update_clear();
                            lane_map_pub.publish(clean_map_);
                            already_drawed_ = false;
                            cout << "----------------------------------- WB: "<< sim_laps << " ------------------------------------" << std::endl;
                            objects_map_file_ << "----------------------------------- WB: "<< sim_laps << " ------------------------------------" << std::endl;
                            fulllog_file_ << "----------------------------------- WB: "<< sim_laps << " ------------------------------------" << std::endl;
                        } else {
                            enable_ctldraw_ = true;
                            cout << "----------------------------------- NB: "<< sim_laps << " ------------------------------------" << std::endl;
                            objects_map_file_ << "----------------------------------- NB: "<< sim_laps << " ------------------------------------" << std::endl;
                            fulllog_file_ << "----------------------------------- NB: "<< sim_laps << " ------------------------------------" << std::endl;
                        }
                        //-----------------------------------------------------------------------------------
                        cout << "----------------------------------- END OF LAP: "<< sim_laps << " ------------------------------------" << std::endl;
                        objects_map_file_ << "----------------------------------- END OF LAP: "<< sim_laps << " ------------------------------------" << std::endl;
                        fulllog_file_ << "----------------------------------- END OF LAP: "<< sim_laps << " ------------------------------------" << std::endl;
                        sim_laps++;
                        if (enable_ctldraw_) {
                            if (!already_drawed_) {
                                cout << "###################ENTREI NA PARTE DO DRAW####################" << endl;
                                enable_ctldraw_obstacles();
                                grid_update_custom();
                                int c = 0;
                                while (c < 30) {
                                    lane_map_pub.publish(lane_map_);
                                    // map_pub.publish(lane_map_);
                                    c++;
                                }
                                for (int cr = 0; cr < 11; cr++){rate.sleep();}
                                
                                enable_ctldraw_ = false;
                            } 
                            // else {
                            //     clean_map_ = grid_map_;
                            //     grid_update_clear();
                            //     lane_map_pub.publish(clean_map_);
                            //     already_drawed_ = false;
                            // }
                        }
                        enable_patrol_ = false;
                    } else {
                        for (int wait = 0; wait < 20; wait++){rate.sleep();}
                        enable_patrol_ = true;
                    }
                } else {
                    cout << "FINALIZADO O PROCESSO DE SIMULACAO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                    objects_map_file_.close();
                    fulllog_file_.close();
                }

                // //-----------------------------------------------------------------------------------
                // if (setup) {
                //     tie(input_goal_x,input_goal_y) = goals[index_];
                //     cout << "GOAL [" << index_-1 << " -> " << index_ << "] FOR " << (std::string)argv[1] << ": [ " << input_goal_x << " | " << input_goal_y << " ] " << endl;
                //     fulllog_file_ << "GOAL [" << index_-1 << " -> " << index_ << "] FOR " << (std::string)argv[1] << ": [ " << input_goal_x << " | " << input_goal_y << " ] " << endl;
                //     goals_output.target_pose.header.frame_id = "map";
                //     goals_output.target_pose.pose.position.x = input_goal_x;
                //     goals_output.target_pose.pose.position.y = input_goal_y;
                //     goals_output.target_pose.pose.position.z = 0;
                //     goals_output.target_pose.pose.orientation.x = 0.0;
                //     goals_output.target_pose.pose.orientation.y = 0.0;
                //     goals_output.target_pose.pose.orientation.z = 0.25;
                //     goals_output.target_pose.pose.orientation.w = 0.95;

                //     move_base_client_.sendGoal(goals_output);
                //     last_index = index_;
                //     index_++;
                //     setup = false;
                //     if (!time_started_) {
                //         time_started_ = true;
                //         start_time_ = std::chrono::steady_clock::now();
                //     }
                // } else {
                //     if (enable_patrol_) {
                //         if (move_base_client_.waitForResult()) {
                //             // if (finished_lap) {
                //             //     finished_lap = false;
                //             //     //--------------------------ADICIONADO 03/01/23--------------------------------------
                //             //     // enable_ctldraw_ = true;
                //             //     //-----------------------------------------------------------------------------------
                //             //     if (sim_laps < 10) {
                //             //         //--------------------------ADICIONADO 05/01/23--------------------------------------
                //             //         if (sim_laps % 2 == 0) {
                //             //             clean_map_ = grid_map_;
                //             //             grid_update_clear();
                //             //             lane_map_pub.publish(clean_map_);
                //             //             already_drawed_ = false;
                //             //         } else {
                //             //             enable_ctldraw_ = true;
                //             //         }
                //             //         //-----------------------------------------------------------------------------------
                //             //         cout << "----------------------------------- END OF LAP: "<< sim_laps << " ------------------------------------" << std::endl;
                //             //         objects_map_file_ << "----------------------------------- END OF LAP: "<< sim_laps << " ------------------------------------" << std::endl;
                //             //         fulllog_file_ << "----------------------------------- END OF LAP: "<< sim_laps << " ------------------------------------" << std::endl;
                //             //         sim_laps++;
                //             //     } else {
                //             //         cout << "FINALIZADO O PROCESSO DE SIMULACAO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                //             //         objects_map_file_.close();
                //             //         fulllog_file_.close();
                //             //     }
                //             // }
                //             cout << "TO AQUI COM O INDEX: " << index_ << endl;
                //             tie(input_goal_x,input_goal_y) = goals[index_];
                            
                //             goals_output.target_pose.header.frame_id = "map";
                //             goals_output.target_pose.pose.position.x = input_goal_x;
                //             goals_output.target_pose.pose.position.y = input_goal_y;
                //             goals_output.target_pose.pose.position.z = 0;
                //             goals_output.target_pose.pose.orientation.x = 0.0;
                //             goals_output.target_pose.pose.orientation.y = 0.0;
                //             goals_output.target_pose.pose.orientation.z = 0.25;
                //             goals_output.target_pose.pose.orientation.w = 0.95;

                //             // int cont = 0;
                //             // while (cont < 3500) {
                //                 // cont++;
                //             // }
                //             // if (cont >= 3499) {
                //                 cout << "MANDEI O PROXIMO GOAL" << endl;
                //                 move_base_client_.sendGoal(goals_output);
                //             // }

                //             if (!time_started_) {
                //                 time_started_ = true;
                //                 start_time_ = std::chrono::steady_clock::now();
                //             } else {
                //                 end_time_ = std::chrono::steady_clock::now();
                //                 // time_started_ = false;
                //                 std::cout << "TERMINEI VOU GRAVAR - GOAL [" << last_index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time_ - start_time_).count() << "[s]" << std::endl;
                //                 write_in_file(last_index,last_index-1,start_time_,end_time_);
                //                 start_time_ = std::chrono::steady_clock::now();
                //             }
                //             cout << "LAST_INDEX: " << last_index << " INDEX_: " << index_ << endl;
                //             fulllog_file_ << "LAST_INDEX: " << last_index << " INDEX_: " << index_ << endl;
                //             cout << "GOAL [" << last_index << " -> " << index_ << "] FOR HUSKY: [ " << input_goal_x << " | " << input_goal_y << " ] " << endl;
                //             fulllog_file_ << "GOAL [" << last_index << " -> " << index_ << "] FOR HUSKY: [ " << input_goal_x << " | " << input_goal_y << " ] " << endl;

                //             last_index = index_;
                //             if (last_index > 2){
                //                 first_setup = true;
                //             }
                //             index_++;
                //             if (index_ > goals.size()-1) {
                //                 index_ = 0;
                //                 finished_lap = true;
                //             }
                //             if (last_index == goals.size()-1 && finished_lap) {
                //                 finished_lap = false;
                //                 //--------------------------ADICIONADO 03/01/23--------------------------------------
                //                 // enable_ctldraw_ = true;
                //                 //--------------------------ADICIONADO 06/01/23--------------------------------------
                //                 enable_patrol_ = false;
                //                 //-----------------------------------------------------------------------------------
                //                 if (sim_laps < 10) {
                //                     //--------------------------ADICIONADO 05/01/23--------------------------------------
                //                     if (sim_laps % 2 == 0) {
                //                         clean_map_ = grid_map_;
                //                         grid_update_clear();
                //                         lane_map_pub.publish(clean_map_);
                //                         already_drawed_ = false;
                //                     } else {
                //                         enable_ctldraw_ = true;
                //                     }
                //                     //-----------------------------------------------------------------------------------
                //                     cout << "----------------------------------- END OF LAP: "<< sim_laps << " ------------------------------------" << std::endl;
                //                     objects_map_file_ << "----------------------------------- END OF LAP: "<< sim_laps << " ------------------------------------" << std::endl;
                //                     fulllog_file_ << "----------------------------------- END OF LAP: "<< sim_laps << " ------------------------------------" << std::endl;
                //                     sim_laps++;
                //                 } else {
                //                     cout << "FINALIZADO O PROCESSO DE SIMULACAO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                //                     objects_map_file_.close();
                //                     fulllog_file_.close();
                //                 }
                //             }
                //             // index_++;
                //             // if (index_ > goals.size()-1) {
                //             //     index_ = 0;
                //             //     finished_lap = true;
                //             // }
                //         }
                //     }
                // }
                // cout << "FIMMMMMMMMMMMMMMMMMMMMMMMMMMMM" << endl;
            } 
            //--------------------------ADICIONADO 03/01/23--------------------------------------
            // else {
            //     if (disable_all_ctldraw_) {
            //         clean_map_ = grid_map_;
            //         grid_update_clear();
            //         lane_map_pub.publish(clean_map_);
            //         disable_all_ctldraw_ = false;
            //     }
            // }
            //-----------------------------------------------------------------------------------
        }
        }else{
            // cout << "AINDA NAO RECEBI O MAPA" << endl;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}