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
#include <move_base_msgs/MoveBaseGoal.h>

using namespace std::chrono;
using namespace std;

// vector<tuple<float,float>> goals;
// int index_ = 0;

bool time_started_ = false;
std::chrono::steady_clock::time_point start_time_;
std::chrono::steady_clock::time_point end_time_;

move_base_msgs::MoveBaseGoal goals_output_;
bool is_object_to_clean_ = false;
bool move_base_started = false;
bool object_cleaned_ = false;
std_msgs::Bool object_cleaned_topic_;

void objects_goal_to_remove_callback(const move_base_msgs::MoveBaseGoal::ConstPtr& object_goal_msg){
    cout << "TENHO OBJETO NO MAPA PARA LIMPAR!" << endl;
    goals_output_.target_pose.header.frame_id = object_goal_msg->target_pose.header.frame_id;
    goals_output_.target_pose.header.seq = object_goal_msg->target_pose.header.seq;
    goals_output_.target_pose.header.stamp = object_goal_msg->target_pose.header.stamp;
    goals_output_.target_pose.pose.position.x = object_goal_msg->target_pose.pose.position.x;
    goals_output_.target_pose.pose.position.y = object_goal_msg->target_pose.pose.position.y;
    goals_output_.target_pose.pose.position.z = object_goal_msg->target_pose.pose.position.z;
    goals_output_.target_pose.pose.orientation.x = object_goal_msg->target_pose.pose.orientation.x;
    goals_output_.target_pose.pose.orientation.y = object_goal_msg->target_pose.pose.orientation.y;
    goals_output_.target_pose.pose.orientation.z = object_goal_msg->target_pose.pose.orientation.z;
    goals_output_.target_pose.pose.orientation.w = object_goal_msg->target_pose.pose.orientation.w;
    is_object_to_clean_ = true;
    move_base_started = true;
    object_cleaned_ = false;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "object_to_remove");
    ros::NodeHandle node;

    ros::Subscriber new_object_to_remove_sub = node.subscribe("/objects_goal_to_remove",1,objects_goal_to_remove_callback);

    ros::Publisher clean_object_from_map_pub = node.advertise<std_msgs::Bool>("/clean_object_from_map",10);

    std::stringstream move_base_topic_stream;
    move_base_topic_stream << "/" << (std::string)argv[1] << "/move_base";
    std::string move_base_topic = move_base_topic_stream.str();

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_(move_base_topic);

    ros::Rate rate(10);

    while(ros::ok()){

        if (!move_base_client_.isServerConnected()){
            ros::spinOnce();
            rate.sleep();
        } else {
            if (is_object_to_clean_) {
                cout << "MANDANDO ROBO PARA O PONTO NO QUAL ELE DEVE LIMPAR O OBJETO!" << endl;
                move_base_client_.sendGoal(goals_output_);
                is_object_to_clean_ = false;
            }

            if (move_base_started) {
                if (move_base_client_.waitForResult() && !is_object_to_clean_ && !object_cleaned_) {         
                    object_cleaned_topic_.data = true;
                    clean_object_from_map_pub.publish(object_cleaned_topic_);
                    cout << "CHEGUEI NO OBJETO E JA LIMPEI ELE" << endl;
                    object_cleaned_ = true;
                }
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}