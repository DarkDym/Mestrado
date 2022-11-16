#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include <string>
#include <move_base_msgs/MoveBaseGoal.h>
#include <std_msgs/Bool.h>

using namespace std;

move_base_msgs::MoveBaseGoal goals_output_;
bool object_cleaned_from_map_ = false;
bool start_object_cleaned_from_map_ = false;
bool object_demarked_from_map_ = false;
bool setup = false;
vector<tuple<int,float,float>> object_goals_;
fstream objects_file_;

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

void clean_object_from_map_callback(const std_msgs::Bool& clean_object_msg){
    object_cleaned_from_map_ = clean_object_msg.data;
}

void start_clean_object_from_map_callback(const std_msgs::Bool& clean_object_msg){
    start_object_cleaned_from_map_ = clean_object_msg.data;
}

void object_demarked_from_map_callback(const std_msgs::Bool& demarked_from_map_msg){
    object_demarked_from_map_ = demarked_from_map_msg.data;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "object_goal_reader");
    ros::NodeHandle node;

    open_file();
    read_file();

    ros::Subscriber clean_object_from_map_sub = node.subscribe("/clean_object_from_map",10,clean_object_from_map_callback);
    ros::Subscriber start_clean_object_from_map_sub = node.subscribe("/start_clean_object_from_map",10,start_clean_object_from_map_callback);
    ros::Subscriber object_demarked_from_map_sub = node.subscribe("/object_demarked_from_map",10,object_demarked_from_map_callback);
    
    ros::Publisher new_object_to_remove_pub = node.advertise<move_base_msgs::MoveBaseGoal>("/objects_goal_to_remove",1);
    ros::Publisher all_objects_analyzed_pub = node.advertise<std_msgs::Bool>("/all_objects_analyzed",10);

    ros::Rate rate(10);

    int cont_control = 0;
    int cell;
    float px,py;

    cout << "QUANTIDADE DE GOALS: " << object_goals_.size() << endl;

    while(ros::ok()){
        while(cont_control < object_goals_.size()){
            // cout << "OBJETO QUE VAI SER ANALISADO: " << cont_control << endl;
            if (!setup) {
                tie(cell,px,py) = object_goals_[cont_control];
                goals_output_.target_pose.header.frame_id = "map";
                goals_output_.target_pose.header.stamp = ros::Time::now();
                goals_output_.target_pose.pose.position.x = px-1.0;
                goals_output_.target_pose.pose.position.y = py-1.0;
                goals_output_.target_pose.pose.position.z = 0.00;
                goals_output_.target_pose.pose.orientation.x = 0.00;
                goals_output_.target_pose.pose.orientation.y = 0.00;
                goals_output_.target_pose.pose.orientation.z = 0.25;
                goals_output_.target_pose.pose.orientation.w = 0.95;
                new_object_to_remove_pub.publish(goals_output_);
                if (start_object_cleaned_from_map_) {
                    cont_control++;
                    setup = true;
                }
            } else {
                // cout << "object_cleaned_from_map_: " << object_cleaned_from_map_ << " | object_demarked_from_map_:" << object_demarked_from_map_ << endl;
                if (object_cleaned_from_map_ && object_demarked_from_map_) {
                    tie(cell,px,py) = object_goals_[cont_control];
                    goals_output_.target_pose.header.frame_id = "map";
                    goals_output_.target_pose.header.stamp = ros::Time::now();
                    goals_output_.target_pose.pose.position.x = px-1.0;
                    goals_output_.target_pose.pose.position.y = py-1.0;
                    goals_output_.target_pose.pose.position.z = 0.00;
                    goals_output_.target_pose.pose.orientation.x = 0.00;
                    goals_output_.target_pose.pose.orientation.y = 0.00;
                    goals_output_.target_pose.pose.orientation.z = 0.25;
                    goals_output_.target_pose.pose.orientation.w = 0.95;
                    new_object_to_remove_pub.publish(goals_output_);
                    cont_control++;
                    object_cleaned_from_map_ = false;
                }
            }
            ros::spinOnce();
            rate.sleep();  
        }
        std_msgs::Bool all_objects;
        all_objects.data = true;
        all_objects_analyzed_pub.publish(all_objects);
        // if (cont_control < object_goals_.size()) {
            // cout << "OS GOALS DO ARQUIVO FORAM FINALIZADOS!!!!!" << endl;
            // cout << "QUANTIDADE DE GOALS: " << object_goals_.size() << endl;
            // cout << "CONT_CONTROL: " << cont_control << endl;
        // }    
        ros::spinOnce();
        rate.sleep();   
    }

    return 0;
}