import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class TransferData:
    def __init__(self):
        rospy.init_node('transfer_data_node', anonymous=True)
        self.pub_odom = rospy.Publisher('/wheel_odom', Odometry, queue_size=100)
        rospy.Subscriber('/robot_velocity', Twist, self.velocityCallback)

    def velocityCallback(self, msg):
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # 👉 ตั้งค่า Twist จาก input
        odom_msg.twist.twist = msg

        # ✅ ใส่ค่าความเชื่อมั่นของ twist (velocity)
        odom_msg.twist.covariance = [
            0.01, 0,    0, 0, 0, 0,
            0,    0.01, 0, 0, 0, 0,
            0,    0,    0.1,0, 0, 0,
            0,    0,    0, 0.1,0, 0,
            0,    0,    0, 0, 0.1,0,
            0,    0,    0, 0, 0, 0.2
        ]

        # ✅ เพิ่มค่าความเชื่อมั่นของ pose (แม้จะไม่มีตำแหน่งจริง)
        odom_msg.pose.covariance = [
            0.01, 0,    0, 0, 0, 0,
            0,    0.01, 0, 0, 0, 0,
            0,    0,    0.1,0, 0, 0,
            0,    0,    0, 0.1,0, 0,
            0,    0,    0, 0, 0.1,0,
            0,    0,    0, 0, 0, 0.2
        ]

        # ✅ ใส่ orientation.w ให้ไม่เป็น 0 เพื่อไม่ให้ quaternion invalid
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
