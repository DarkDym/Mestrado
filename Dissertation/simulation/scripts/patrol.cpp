//To be Added
#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>


int main(int argc, char **argv){
    
    ros::init(argc, argv, "patrol_move");
    ros::NodeHandle node;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_("husky1/move_base");

    ros::Rate rate(10);

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}