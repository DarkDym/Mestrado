//To be Added
#include "path_fromtag.h"

using namespace stop_explore_tag;

// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>

#define ROBOT_NAMESPACE = "/husky1"

// namespace map_merge{
    // map_merge::MapMerge map_merge_;
// }

int merge_trigger = 0;
int alredy_merged = 0;

void apriltag_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& tag_msg){
    if (tag_msg->detections.size() > 0){
        ROS_INFO("TAG_CALLBACK: [%f]", tag_msg->detections[0].pose.pose.pose.position.x);
        if (alredy_merged == 0){
            merge_trigger = 1;
        }
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "tag_listener");

    ros::NodeHandle node;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> movebase_client ("/husky1/explore", true);

    ros::Subscriber tag_sub = node.subscribe("/husky1/tag_detections",1000,apriltag_callback);
    

    // movebase_client mbc("/husky1/move_base", true);

    ros::spin();

    // while (alredy_merged == 0){
    //     if (merge_trigger == 1 && alredy_merged == 0){
    //         // map_merge_.mapMerging();
    //         movebase_client.cancelAllGoals();
    //         alredy_merged = 1;
    //         merge_trigger = 0;
    //     }
    // }

    return 0;
}