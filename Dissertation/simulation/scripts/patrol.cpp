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
int last_index = 0;
int index_ = 0;
bool missionStop = false;

void mission_goals(){
    tuple<float,float> inv_goals;    

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

void stopMission(bool stop){
    if (stop) {
        last_index = index_;
        missionStop = true;
    }
}

void resumeMission(bool resume){
    if (resume)
        missionStop = false;
}

void stop_callback(const std_msgs::Bool& stop_msg){
    stopMission(stop_msg.data);
}

void resume_callback(const std_msgs::Bool& resume_msg){
    resumeMission(resume_msg.data);
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "patrol_move");
    ros::NodeHandle node;

    mission_goals();

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_("/husky1/move_base");

    ros::Subscriber stop_mission_sub = node.subscribe("/stop_mission",1,stop_callback);
    ros::Subscriber resume_mission_sub = node.subscribe("/resume_mission",1,resume_callback);

    move_base_msgs::MoveBaseGoal goals_output;

    ros::Rate rate(10);

    // int index = 0;
    float input_goal_x, input_goal_y;
    bool setup = true;

    while(ros::ok()){

        if (!move_base_client_.isServerConnected()){
            ros::spinOnce();
            rate.sleep();
        } else {
            if (!missionStop) {
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
                        if (index_ > 13) {
                            index_ = 0;
                        }
                    }
                }
            } else {
                move_base_client_.cancelAllGoals();
                index_ = last_index;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}