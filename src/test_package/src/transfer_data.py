import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class TransferData:
    def __init__(self):
        rospy.init_node('transfer_data_node', anonymous=True)
        self.pub_odom = rospy.Publisher('/wheel_odom', Odometry, queue_size=100)
        self.br = tf.TransformBroadcaster()
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = rospy.Time.now()
        rospy.Subscriber('/robot_velocity', Twist, self.velocityCallback)

    def velocityCallback(self, msg):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        vx = msg.linear.x
        vy = msg.linear.y  # สำหรับ mecanum wheel
        vth = msg.angular.z

        # 👉 คำนวณตำแหน่งแบบ simple dead-reckoning
        delta_x = (vx * dt * rospy.cos(self.th)) - (vy * dt * rospy.sin(self.th))
        delta_y = (vx * dt * rospy.sin(self.th)) + (vy * dt * rospy.cos(self.th))
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # ✅ ส่ง TF
        self.br.sendTransform(
            (self.x, self.y, 0),
            tf.transformations.quaternion_from_euler(0, 0, self.th),
            current_time,
            "base_link",
            "odom"
        )

        # ✅ ส่ง Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = self._to_orientation(self.th)

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
        self.last_time = current_time

    def _to_orientation(self, yaw):
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        from geometry_msgs.msg import Quaternion
        return Quaternion(*q)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        trans1 = TransferData()
        trans1.run()
    except rospy.ROSInterruptException:
        pass
