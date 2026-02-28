#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf.transformations
import math
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion

class MecanumOdometryNode:
    def __init__(self):
        """
        Constructor for MecanumOdometryNode class.
        """
        rospy.init_node('mecanum_velocity_node')
        rospy.loginfo("Starting Mecanum Odometry Node")

        # === พารามิเตอร์ของหุ่นยนต์ ===
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.04)
        self.l1 = rospy.get_param("~l1", 0.105)
        self.l2 = rospy.get_param("~l2", 0.0825)
        self.ppr = rospy.get_param("~ppr", 660.0)
        
        # <<< CHANGED: ไม่ใช้ update_dt แบบคงที่แล้ว จะใช้ publish_rate แทน
        publish_rate = rospy.get_param("~publish_rate", 25.0) # ความถี่ในการ publish TF และ odom (Hz)
        self.timeout_duration = rospy.Duration(0.5) # ถ้าไม่ได้รับข้อมูล encoder นานกว่านี้ ถือว่าหยุดนิ่ง

        # === ตัวแปรสำหรับเก็บค่า Encoder ===
        self.enc = {'FL': None, 'FR': None, 'RL': None, 'RR': None}
        self.prev_enc = {'FL': None, 'FR': None, 'RL': None, 'RR': None}
        
        # <<< CHANGED: เพิ่มตัวแปรสำหรับเวลา
        self.last_update_time = None

        # === ตัวคูณชดเชยสำหรับแต่ละล้อ ===
        self.scale = {
            'FL': rospy.get_param("~scale_FL", 1.0),
            'FR': rospy.get_param("~scale_FR", 1.0),
            'RL': rospy.get_param("~scale_RL", 1.0),
            'RR': rospy.get_param("~scale_RR", 1.0)
        }

        # === ตัวแปรสำหรับเก็บ Pose และ Velocity ===
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        # === สร้าง Broadcaster สำหรับ TF Transform ===
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # === Publisher และ Subscriber ===
        self.odom_pub = rospy.Publisher("wheel_odom", Odometry, queue_size=10)
        encoders_topic = rospy.get_param("~encoders_topic", "encoders_data")
        rospy.Subscriber(encoders_topic, Int32MultiArray, self.encoders_callback)
        
        # <<< CHANGED: Timer จะเรียกฟังก์ชัน publish_odometry แทน
        rospy.Timer(rospy.Duration(1.0/publish_rate), self.publish_odometry)

    def encoders_callback(self, msg):
        """
        Callback function to receive encoder data and perform all calculations.
        """
        current_time = rospy.Time.now()

        # --- ตรวจสอบข้อมูลและเวลา ---
        if len(msg.data) != 4:
            rospy.logwarn("Received encoder data with incorrect length.")
            return

        if self.last_update_time is None:
            self.last_update_time = current_time
            # กำหนดค่าเริ่มต้นให้กับ prev_enc
            self.prev_enc['FL'] = msg.data[0]
            self.prev_enc['FR'] = msg.data[1]
            self.prev_enc['RL'] = msg.data[2]
            self.prev_enc['RR'] = msg.data[3]
            return

        # <<< CHANGED: คำนวณ dt จากเวลาจริง
        dt = (current_time - self.last_update_time).to_sec()
        if dt == 0:
            return

        self.last_update_time = current_time

        # --- เริ่มการคำนวณ ---
        self.enc['FL'] = msg.data[0]
        self.enc['FR'] = msg.data[1]
        self.enc['RL'] = msg.data[2]
        self.enc['RR'] = msg.data[3]

        # 1. คำนวณผลต่างของ pulse
        delta_pulse = {key: (self.enc[key] - self.prev_enc[key]) * self.scale[key] for key in self.enc}
        self.prev_enc = self.enc.copy()

        # 2. แปลง pulse เป็นความเร็วเชิงมุมของล้อ (rad/s)
        def pulse_to_rad_per_sec(pulse):
            return (pulse / self.ppr) * 2 * math.pi / dt
        
        w = {key: pulse_to_rad_per_sec(delta_pulse[key]) for key in delta_pulse}

        # 3. Inverse Kinematics หาความเร็วหุ่นยนต์ (vx, vy, vth)
        self.vx = (self.wheel_radius / 4.0) * (w['FL'] + w['FR'] + w['RL'] + w['RR'])
        self.vy = (self.wheel_radius / 4.0) * (-w['FL'] + w['FR'] + w['RL'] - w['RR'])
        self.vth = (self.wheel_radius / (4.0 * (self.l1 + self.l2))) * (-w['FL'] + w['FR'] - w['RL'] + w['RR'])

        # 4. คำนวณตำแหน่งใหม่ (Pose Integration)
        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        delta_theta = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

    def publish_odometry(self, event):
        """
        Main function to publish odometry and TF transform periodically.
        Also checks for timeout to reset velocity to zero.
        """
        current_time = rospy.Time.now()

        # <<< CHANGED: ตรวจสอบ Timeout
        if self.last_update_time is not None and (current_time - self.last_update_time) > self.timeout_duration:
            self.vx = 0.0
            self.vy = 0.0
            self.vth = 0.0
            rospy.logdebug("Encoder timeout, setting velocity to zero.")

        # 5. สร้างและส่ง TF Transform จาก odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        q = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation = Quaternion(*q)
        self.tf_broadcaster.sendTransform(t)

        # 6. สร้างและ Publish Odometry Message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(*q)
        odom.pose.covariance = [0.1, 0, 0, 0, 0, 0,
                                0, 0.1, 0, 0, 0, 0,
                                0, 0, 999, 0, 0, 0,
                                0, 0, 0, 999, 0, 0,
                                0, 0, 0, 0, 999, 0,
                                0, 0, 0, 0, 0, 0.1]

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        odom.twist.covariance = [0.01, 0, 0, 0, 0, 0,
                                 0, 0.01, 0, 0, 0, 0,
                                 0, 0, 999, 0, 0, 0,
                                 0, 0, 0, 999, 0, 0,
                                 0, 0, 0, 0, 999, 0,
                                 0, 0, 0, 0, 0, 0.1]
                                 
        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        MecanumOdometryNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mecanum Odometry Node shut down.")