#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg
import tf_conversions

def publish_static_tf(parent, child, x, y, z, roll, pitch, yaw):
    br = tf2_ros.StaticTransformBroadcaster()
    static_tf = geometry_msgs.msg.TransformStamped()

    static_tf.header.stamp = rospy.Time.now()
    static_tf.header.frame_id = parent
    static_tf.child_frame_id = child

    static_tf.transform.translation.x = x
    static_tf.transform.translation.y = y
    static_tf.transform.translation.z = z

    quat = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
    static_tf.transform.rotation.x = quat[0]
    static_tf.transform.rotation.y = quat[1]
    static_tf.transform.rotation.z = quat[2]
    static_tf.transform.rotation.w = quat[3]

    br.sendTransform(static_tf)

if __name__ == '__main__':
    rospy.init_node('custom_static_tf_publisher')
    rospy.sleep(0.5)  # ให้ ROS พร้อมก่อน

    publish_static_tf("imu_link", "base_link", -0.09, -0.01, -0.24, 0, 0, 0)
    publish_static_tf("mag_link", "base_link", -0.03, -0.02, -0.24, 0, 0, 0)

    rospy.spin()
