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

         # ➕ กำหนดค่าความเชื่อมั่นให้ EKF ยอมรับ
        odom_msg.twist.covariance[0] = 0.01   # linear x
        odom_msg.twist.covariance[7] = 0.01   # linear y
        odom_msg.twist.covariance[35] = 0.01  # angular z

        # ➕ ตั้ง orientation.w ให้ไม่เป็น 0
        odom_msg.pose.pose.orientation.w = 1.0

        self.pub_odom.publish(odom_msg)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        trans1 = TransferData()
        trans1.run()
    except rospy.ROSInterruptException:
        pass