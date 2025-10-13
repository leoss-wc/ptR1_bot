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
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.04)     # รัศมีล้อ (เมตร)
        self.l1 = rospy.get_param("~l1", 0.105)                       # ระยะห่างครึ่งหนึ่งของความยาวหุ่นยนต์
        self.l2 = rospy.get_param("~l2", 0.0825)                      # ระยะห่างครึ่งหนึ่งของความกว้างหุ่นยนต์
        self.ppr = rospy.get_param("~ppr", 660.0)                     # Pulses Per Revolution ของ Encoder
        self.update_dt = rospy.get_param("~update_dt", 0.04)          # ช่วงเวลาในการอัปเดต (วินาที), ควรตรงกับ Arduino

        # === ตัวแปรสำหรับเก็บค่า Encoder ===
        self.enc = {'FL': None, 'FR': None, 'RL': None, 'RR': None}
        self.prev_enc = {'FL': None, 'FR': None, 'RL': None, 'RR': None}

        # === ตัวคูณชดเชยสำหรับแต่ละล้อ (ถ้าจำเป็น) ===
        self.scale = {
            'FL': rospy.get_param("~scale_FL", 1.0),
            'FR': rospy.get_param("~scale_FR", 1.0),
            'RL': rospy.get_param("~scale_RL", 1.0),
            'RR': rospy.get_param("~scale_RR", 1.0)
        }

        # === ตัวแปรสำหรับเก็บตำแหน่งและมุมของหุ่นยนต์ (Pose) ===
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # === สร้าง Broadcaster สำหรับ TF Transform ===
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # === Publisher และ Subscriber ===
        self.odom_pub = rospy.Publisher("wheel_odom", Odometry, queue_size=10)
        encoders_topic = rospy.get_param("~encoders_topic", "encoders_data")
        rospy.Subscriber(encoders_topic, Int32MultiArray, self.encoders_callback)
        
        # === ตั้งเวลาทำงานฟังก์ชันอัปเดตหลัก ===
        rospy.Timer(rospy.Duration(self.update_dt), self.update_odometry)

    def encoders_callback(self, msg):
        """
        Callback function to receive encoder data from Arduino.
        """
        if len(msg.data) == 4:
            self.enc['FL'] = msg.data[0]
            self.enc['FR'] = msg.data[1]
            self.enc['RL'] = msg.data[2]
            self.enc['RR'] = msg.data[3]

            # กำหนดค่าเริ่มต้นให้กับ prev_enc ในครั้งแรกที่ได้รับข้อมูล
            for key in self.prev_enc:
                if self.prev_enc[key] is None:
                    self.prev_enc[key] = self.enc[key]
        else:
            rospy.logwarn("Received encoder data with incorrect length.")

    def update_odometry(self, event):
        """
        Main function to calculate and publish odometry and TF transform.
        """
        # รอจนกว่าจะได้รับข้อมูลจาก encoder ครบทุกตัว
        if None in self.enc.values():
            rospy.logwarn_once("Waiting for encoder data...")
            return
        
        current_time = rospy.Time.now()

        # 1. คำนวณผลต่างของ pulse จากครั้งก่อน
        delta_pulse = {key: (self.enc[key] - self.prev_enc[key]) * self.scale[key] for key in self.enc}
        
        # อัปเดตค่า encoder ของครั้งก่อน
        for key in self.enc:
            self.prev_enc[key] = self.enc[key]

        # 2. แปลง pulse เป็นความเร็วเชิงมุมของแต่ละล้อ (rad/s)
        def pulse_to_rad_per_sec(pulse):
            return (pulse / self.ppr) * 2 * math.pi / self.update_dt
        
        wheel_angular_vel = {key: pulse_to_rad_per_sec(delta_pulse[key]) for key in delta_pulse}
        w = wheel_angular_vel # ใช้ชื่อสั้นลง

        # 3. ใช้ Inverse Kinematics ของ Mecanum เพื่อหาความเร็วของหุ่นยนต์ (vx, vy, vth)
        vx = (self.wheel_radius / 4.0) * (w['FL'] + w['FR'] + w['RL'] + w['RR'])
        vy = (self.wheel_radius / 4.0) * (-w['FL'] + w['FR'] + w['RL'] - w['RR'])
        vth = (self.wheel_radius / (4.0 * (self.l1 + self.l2))) * (-w['FL'] + w['FR'] - w['RL'] + w['RR'])

        # 4. คำนวณตำแหน่งใหม่ของหุ่นยนต์ (Pose Integration)
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * self.update_dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * self.update_dt
        delta_theta = vth * self.update_dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # 5. สร้างและส่ง TF Transform จาก odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        q = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

        # 6. สร้างและ Publish Odometry Message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # เติมข้อมูลตำแหน่ง (Pose)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(*q)
        odom.pose.covariance = [0.1, 0, 0, 0, 0, 0,
                                0, 0.1, 0, 0, 0, 0,
                                0, 0, 999, 0, 0, 0,
                                0, 0, 0, 999, 0, 0,
                                0, 0, 0, 0, 999, 0,
                                0, 0, 0, 0, 0, 0.1]

        # เติมข้อมูลความเร็ว (Twist)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
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