#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

def imu_callback(msg):
    try:
        # ใช้ TF listener เพื่อดึง transform ล่าสุด
        (trans, rot) = listener.lookupTransform("imu_link", "base_link", rospy.Time(0))

        rospy.loginfo("TF (imu_link -> base_link):")
        rospy.loginfo("  Translation: %s", trans)
        rospy.loginfo("  Rotation: %s", rot)

        # >>>> คุณสามารถใช้ข้อมูลนี้ประมวลผลเพิ่มเติมได้ที่นี่ <<<<

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn("TF lookup failed: %s", e)

def mag_callback(msg):
    try:
        (trans, rot) = listener.lookupTransform("mag_link", "base_link", rospy.Time(0))
        rospy.loginfo("TF (mag_link -> base_link):")
        rospy.loginfo("  Translation: %s", trans)
        rospy.loginfo("  Rotation: %s", rot)

        # >>>> ประมวลผลค่าจาก msg กับ TF ได้ที่นี่ <<<<

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn("TF lookup (mag_link -> base_link) failed: %s", e)

if __name__ == '__main__':
    rospy.init_node('imu_tf_listener')
    listener = tf.TransformListener()

    # subscribe ข้อมูล IMU จาก topic ที่คุณใช้จริง เช่น /imu/data_raw
    rospy.Subscriber("/imu/data_raw", Imu, imu_callback)
    # subscribe ข้อมูล MagneticField จาก topic ที่คุณใช้จริง เช่น /imu/mag
    rospy.Subscriber("/imu/mag", MagneticField, mag_callback)


    rospy.spin()
