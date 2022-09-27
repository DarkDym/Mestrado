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

void mission_goals(){
    tuple<float,float> inv_goals;
    float input_goal_x, input_goal_y;
    fulllog_file_ << "-----------GOALS--------------" << endl;    

    inv_goals = make_tuple(16.83527374267578,-4.076648712158203);
    goals.emplace_back(inv_goals);
    tie(input_goal_x,input_goal_y) = goals[0];
    fulllog_file_ << "GOAL 0: [" << input_goal_x << " | " << input_goal_y << "]" << endl;

    inv_goals = make_tuple(16.635780334472656,2.5355117321014404);
    goals.emplace_back(inv_goals);
    tie(input_goal_x,input_goal_y) = goals[1];
    fulllog_file_ << "GOAL 1: [" << input_goal_x << " | " << input_goal_y << "]" << endl;

    inv_goals = make_tuple(10.314708709716797,43.58147430419922);
    goals.emplace_back(inv_goals);
    tie(input_goal_x,input_goal_y) = goals[2];
    fulllog_file_ << "GOAL 2: [" << input_goal_x << " | " << input_goal_y << "]" << endl;

    inv_goals = make_tuple(13.89610767364502,2.775299072265625);
    goals.emplace_back(inv_goals);
    tie(input_goal_x,input_goal_y) = goals[3];
    fulllog_file_ << "GOAL 3: [" << input_goal_x << " | " << input_goal_y << "]" << endl;

    inv_goals = make_tuple(13.953405380249023,43.73188781738281);
    goals.emplace_back(inv_goals);
    tie(input_goal_x,input_goal_y) = goals[4];
    fulllog_file_ << "GOAL 4: [" << input_goal_x << " | " << input_goal_y << "]" << endl;

    inv_goals = make_tuple(12.793737411499023,2.642454147338867);
    goals.emplace_back(inv_goals);
    tie(input_goal_x,input_goal_y) = goals[5];
    fulllog_file_ << "GOAL 5: [" << input_goal_x << " | " << input_goal_y << "]" << endl;

    fulllog_file_ << "-----------GOALS--------------" << endl;

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
    // return objects_map_file;
}

void init_fulllog_file(std::string arq_name){

    std::stringstream name_stream;
    name_stream << "./full_log_" << arq_name << ".txt";
    std::string file_name = name_stream.str();
    fulllog_file_.open(file_name,ios::app);
    // return objects_map_file;
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
}

void write_in_file(int index, int last_index, std::chrono::steady_clock::time_point start_time, std::chrono::steady_clock::time_point end_time){
    if (objects_map_file_.is_open()) {
        if (last_index < 2 && first_setup) {
            last_index = 5;
            std::cout << "GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() << "[s]" << std::endl;
            objects_map_file_ << "GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() << "[s]" << std::endl;
            fulllog_file_ << "GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() << "[s]" << std::endl;
        } else {
            std::cout << "GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() << "[s]" << std::endl;
            objects_map_file_ << "GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() << "[s]" << std::endl;
            fulllog_file_ << "GOAL [" << last_index << "-> " << index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() << "[s]" << std::endl;
        }
        
        // objects_map_file.close();
    }else{
        cout << "POR ALGUM MOTIVO O ARQUIVO NAO PODE SER ABERTO" << endl;
    }
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "SOWDC_move");
    ros::NodeHandle node;
    // std::ofstream sim_time_file;

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

    ros::Rate rate(10);

    float input_goal_x, input_goal_y;
    bool setup = true;
    int last_index = 0;
    int sim_laps = 0;
    bool finished_lap = false;

    while(ros::ok()){

        if (!move_base_client_.isServerConnected()){
            ros::spinOnce();
            rate.sleep();
        } else {
            if (setup) {
                tie(input_goal_x,input_goal_y) = goals[index_];
                cout << "GOAL [" << index_-1 << " -> " << index_ << "] FOR " << (std::string)argv[1] << ": [ " << input_goal_x << " | " << input_goal_y << " ] " << endl;
                fulllog_file_ << "GOAL [" << index_-1 << " -> " << index_ << "] FOR " << (std::string)argv[1] << ": [ " << input_goal_x << " | " << input_goal_y << " ] " << endl;
                goals_output.target_pose.header.frame_id = "map";
                goals_output.target_pose.pose.position.x = input_goal_x;
                goals_output.target_pose.pose.position.y = input_goal_y;
                goals_output.target_pose.pose.position.z = 0;
                goals_output.target_pose.pose.orientation.x = 0.0;
                goals_output.target_pose.pose.orientation.y = 0.0;
                goals_output.target_pose.pose.orientation.z = 0.25;
                goals_output.target_pose.pose.orientation.w = 0.95;

                move_base_client_.sendGoal(goals_output);
                last_index = index_;
                index_++;
                setup = false;
                if (!time_started_) {
                    time_started_ = true;
                    start_time_ = std::chrono::steady_clock::now();
                }
            } else {
                if (move_base_client_.waitForResult()) {
                    tie(input_goal_x,input_goal_y) = goals[index_];
                    
                    goals_output.target_pose.header.frame_id = "map";
                    goals_output.target_pose.pose.position.x = input_goal_x;
                    goals_output.target_pose.pose.position.y = input_goal_y;
                    goals_output.target_pose.pose.position.z = 0;
                    goals_output.target_pose.pose.orientation.x = 0.0;
                    goals_output.target_pose.pose.orientation.y = 0.0;
                    goals_output.target_pose.pose.orientation.z = 0.25;
                    goals_output.target_pose.pose.orientation.w = 0.95;

                    move_base_client_.sendGoal(goals_output);
                    if (!time_started_) {
                        time_started_ = true;
                        start_time_ = std::chrono::steady_clock::now();
                    } else {
                        end_time_ = std::chrono::steady_clock::now();
                        // time_started_ = false;
                        std::cout << "TERMINEI VOU GRAVAR - GOAL [" << last_index << "]" << " | Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end_time_ - start_time_).count() << "[s]" << std::endl;
                        write_in_file(last_index,last_index-1,start_time_,end_time_);
                        start_time_ = std::chrono::steady_clock::now();
                    }
                    cout << "LAST_INDEX: " << last_index << " INDEX_: " << index_ << endl;
                    fulllog_file_ << "LAST_INDEX: " << last_index << " INDEX_: " << index_ << endl;
                    cout << "GOAL [" << last_index << " -> " << index_ << "] FOR HUSKY: [ " << input_goal_x << " | " << input_goal_y << " ] " << endl;
                    fulllog_file_ << "GOAL [" << last_index << " -> " << index_ << "] FOR HUSKY: [ " << input_goal_x << " | " << input_goal_y << " ] " << endl;

                    last_index = index_;
                    if (last_index > 2){
                        first_setup = true;
                    }
                    if (last_index == 3 && finished_lap) {
                        finished_lap = false;
                        if (sim_laps < 10) {
                            cout << "----------------------------------- END OF LAP: "<< sim_laps << " ------------------------------------" << std::endl;
                            objects_map_file_ << "----------------------------------- END OF LAP: "<< sim_laps << " ------------------------------------" << std::endl;
                            fulllog_file_ << "----------------------------------- END OF LAP: "<< sim_laps << " ------------------------------------" << std::endl;
                            sim_laps++;
                        } else {
                            cout << "FINALIZADO O PROCESSO DE SIMULACAO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                            objects_map_file_.close();
                            fulllog_file_.close();
                        }
                    }
                    index_++;
                    if (index_ > goals.size()-1) {
                        index_ = 0;
                        finished_lap = true;
                    }
                }
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}