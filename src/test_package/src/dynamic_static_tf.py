#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg

def create_static_tf(child, parent, x, y, z):
    tf = geometry_msgs.msg.TransformStamped()
    tf.header.frame_id = parent        # ✅ base_link เป็น parent
    tf.child_frame_id = child          # ✅ imu_link เป็น child
    tf.transform.translation.x = x
    tf.transform.translation.y = y
    tf.transform.translation.z = z
    tf.transform.rotation.x = 0.0
    tf.transform.rotation.y = 0.0
    tf.transform.rotation.z = 0.0
    tf.transform.rotation.w = 1.0
    return tf

if __name__ == '__main__':
    rospy.init_node('custom_static_tf_publisher')
    br = tf2_ros.StaticTransformBroadcaster()
    rate = rospy.Rate(1)

    # ✅ ส่ง TF: base_link → imu_link (x = +0.09 → imu อยู่ด้านหน้าฐาน)
    imu_tf = create_static_tf("imu_link", "base_link", 0.09, 0.01, 0.24)

    # ✅ ส่ง TF: base_link → mag_link
    mag_tf = create_static_tf("mag_link", "base_link", 0.03, 0.02, 0.24)

    rospy.sleep(1.0)

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        imu_tf.header.stamp = now
        mag_tf.header.stamp = now
        br.sendTransform([imu_tf, mag_tf])
        rate.sleep()
