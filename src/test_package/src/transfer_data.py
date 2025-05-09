import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class TransferData:
    def __init__(self):
        rospy.init_node('transfer_data_node', anonymous=True)
        self.pub_odom = rospy.Publisher('/wheel_odom', Odometry, queue_size=100)
        rospy.Subscriber('/robot_velocity', Twist, self.velocityCallback)

    def velocityCallback(self, msg):
        current_time = rospy.Time.now()
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # ✅ ใส่เฉพาะข้อมูลความเร็ว
        odom_msg.twist.twist = msg

        # ✅ อาจใส่ covariance ด้วยถ้าต้องการ
        odom_msg.twist.covariance = [
            0.01, 0,    0, 0, 0, 0,
            0,    0.01, 0, 0, 0, 0,
            0,    0,    0.1,0, 0, 0,
            0,    0,    0, 0.1,0, 0,
            0,    0,    0, 0, 0.1,0,
            0,    0,    0, 0, 0, 0.2
        ]

        self.pub_odom.publish(odom_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        TransferData().run()
    except rospy.ROSInterruptException:
        pass
