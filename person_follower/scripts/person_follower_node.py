import rospy
from std_msgs.msg import String
from diffbot.person_follower.follower import Follower

class PersonFollowerNode:
    def __init__(self):
        rospy.init_node('person_follower_node', anonymous=True)
        self.follower = Follower()
        self.subscriber = rospy.Subscriber('person_detection', String, self.person_detection_callback)
        
    def person_detection_callback(self, msg):
        rospy.loginfo("Detected person: %s", msg.data)
        self.follower.follow_person(msg.data)
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PersonFollowerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass