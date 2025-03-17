import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class TransferData:
    def __init__(self):
        rospy.init_node('transfer_data_node', anonymous=True)
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=100)

        rospy.Subscriber('/robot_velocity', Twist, self.velocityCallback)

    def velocityCallback(self, msg):
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.twist.twist = msg

        self.pub_odom.publish(odom_msg)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        trans1 = TransferData()
        trans1.run()
    except rospy.ROSInterruptException:
        pass