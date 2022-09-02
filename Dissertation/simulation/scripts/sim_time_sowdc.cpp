#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <std_msgs/Bool.h>


using namespace std;

vector<tuple<float,float>> goals;
int index_ = 0;

void mission_goals(){
    tuple<float,float> inv_goals;    

    inv_goals = make_tuple(16.83527374267578,-4.076648712158203);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(10.314708709716797,43.58147430419922);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(13.89610767364502,2.775299072265625);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(13.953405380249023,43.73188781738281);
    goals.emplace_back(inv_goals);

    inv_goals = make_tuple(12.793737411499023,2.642454147338867);
    goals.emplace_back(inv_goals);

}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "SOWDC_move");
    ros::NodeHandle node;

    mission_goals();

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_("/husky1/move_base");

    move_base_msgs::MoveBaseGoal goals_output;

    ros::Rate rate(10);

    float input_goal_x, input_goal_y;
    bool setup = true;

    while(ros::ok()){

        if (!move_base_client_.isServerConnected()){
            ros::spinOnce();
            rate.sleep();
        } else {
            if (setup) {
                tie(input_goal_x,input_goal_y) = goals[index_];

                goals_output.target_pose.header.frame_id = "husky1_tf/map";
                goals_output.target_pose.pose.position.x = input_goal_x;
                goals_output.target_pose.pose.position.y = input_goal_y;
                goals_output.target_pose.pose.position.z = 0;
                goals_output.target_pose.pose.orientation.x = 0.0;
                goals_output.target_pose.pose.orientation.y = 0.0;
                goals_output.target_pose.pose.orientation.z = 0.25;
                goals_output.target_pose.pose.orientation.w = 0.95;

                move_base_client_.sendGoal(goals_output);
                index_++;
                setup = false;
            } else {
                if (move_base_client_.waitForResult()) {
                    tie(input_goal_x,input_goal_y) = goals[index_];
                    
                    cout << "NEXT GOAL [" << index_ << "] FOR HUSKY: [ " << input_goal_x << " | " << input_goal_y << " ] " << endl;
                    
                    goals_output.target_pose.header.frame_id = "husky1_tf/map";
                    goals_output.target_pose.pose.position.x = input_goal_x;
                    goals_output.target_pose.pose.position.y = input_goal_y;
                    goals_output.target_pose.pose.position.z = 0;
                    goals_output.target_pose.pose.orientation.x = 0.0;
                    goals_output.target_pose.pose.orientation.y = 0.0;
                    goals_output.target_pose.pose.orientation.z = 0.25;
                    goals_output.target_pose.pose.orientation.w = 0.95;

                    move_base_client_.sendGoal(goals_output);
                    index_++;
                    if (index_ > 5) {
                        index_ = 0;
                    }
                }
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}