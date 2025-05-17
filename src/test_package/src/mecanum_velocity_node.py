#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
from geometry_msgs.msg import Twist

class MecanumVelocityNode:
    def __init__(self):
        rospy.init_node('mecanum_velocity_node')

        # Robot parameters
        self.r = 0.04  # wheel radius (meters)
        self.l1 = 0.1075  # center to left/right
        self.l2 = 0.0825  # center to front/back
        self.PPR = 660.0  # pulses per revolution
        self.dt = 0.02  # 50 Hz

        # Previous encoder values
        self.prev_FL = None
        self.prev_FR = None
        self.prev_RL = None
        self.prev_RR = None

        # Current encoder values
        self.enc_FL = None
        self.enc_FR = None
        self.enc_RL = None
        self.enc_RR = None

        # Subscribers
        rospy.Subscriber("encoder1", UInt32, self.enc_cb_FL)
        rospy.Subscriber("encoder2", UInt32, self.enc_cb_FR)
        rospy.Subscriber("encoder3", UInt32, self.enc_cb_RL)
        rospy.Subscriber("encoder4", UInt32, self.enc_cb_RR)

        # Publisher
        self.vel_pub = rospy.Publisher("/robot_velocity_raspi", Twist, queue_size=10)

        # Timer
        rospy.Timer(rospy.Duration(self.dt), self.update_velocity)

    def enc_cb_FL(self, msg): self.enc_FL = msg.data
    def enc_cb_FR(self, msg): self.enc_FR = msg.data
    def enc_cb_RL(self, msg): self.enc_RL = msg.data
    def enc_cb_RR(self, msg): self.enc_RR = msg.data

    def update_velocity(self, event):
        if None in [self.enc_FL, self.enc_FR, self.enc_RL, self.enc_RR]:
            return  # ยังไม่มีข้อมูลครบ 4 ล้อ

        # แปลงเป็น delta pulse
        if None in [self.prev_FL, self.prev_FR, self.prev_RL, self.prev_RR]:
            self.prev_FL, self.prev_FR = self.enc_FL, self.enc_FR
            self.prev_RL, self.prev_RR = self.enc_RL, self.enc_RR
            return

        dFL = self.enc_FL - self.prev_FL
        dFR = self.enc_FR - self.prev_FR
        dRL = self.enc_RL - self.prev_RL
        dRR = self.enc_RR - self.prev_RR

        self.prev_FL, self.prev_FR = self.enc_FL, self.enc_FR
        self.prev_RL, self.prev_RR = self.enc_RL, self.enc_RR

        # pulse → angular velocity (rad/s)
        w_FL = (dFL / self.PPR) * 2 * 3.14159265 / self.dt
        w_FR = (dFR / self.PPR) * 2 * 3.14159265 / self.dt
        w_RL = (dRL / self.PPR) * 2 * 3.14159265 / self.dt
        w_RR = (dRR / self.PPR) * 2 * 3.14159265 / self.dt

        # Mecanum inverse kinematics
        vx = (self.r / 4.0) * (w_FL + w_FR + w_RL + w_RR)
        vy = (self.r / 4.0) * (-w_FL + w_FR + w_RL - w_RR)
        omega = (self.r / (4.0 * (self.l1 + self.l2))) * (-w_FL + w_FR - w_RL + w_RR)

        # Publish
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = omega
        self.vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        MecanumVelocityNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
