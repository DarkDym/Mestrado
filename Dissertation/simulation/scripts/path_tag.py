import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from move_base_msgs.msg import MoveBaseAction
import actionlib

ROBOT_NAMESPACE = "/husky1"

class TagRead:
    def __init__(self):
        print("#####################################INITIALIZE SCRIPT#####################################")
        
        self.movebase_client = actionlib.SimpleActionClient(ROBOT_NAMESPACE+"/move_base",MoveBaseAction)

        rospy.Subscriber(ROBOT_NAMESPACE+"/tag_detections", AprilTagDetectionArray, self.tag_callback)
        # rospy.spin()
        
    
    def tag_callback(self,tag_msg):
        print("##############ACHEI A TAG##############")
        print(tag_msg.detections[0].id[0])

if __name__ == '__main__':
    
    rospy.init_node("listener_tag")

    tg_read = TagRead()