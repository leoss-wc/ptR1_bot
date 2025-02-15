#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, UInt32, UInt16, UInt8
from geometry_msgs.msg import Twist

class CommandHandler:
    def __init__(self):
        rospy.init_node('command_node', anonymous=True)
        self.pub_motor = rospy.Publisher('/arduino/drive', UInt16, queue_size=100)
        self.pub_servo = rospy.Publisher('/arduino/servo', UInt8, queue_size=100)
        self.pub_edit = rospy.Publisher('/arduino/edit', UInt32, queue_size=100)
        self.manual_drive_data = None  # เก็บคำสั่งจาก /rb/cm/dr
        self.auto_drive_data = None    # เก็บคำสั่งจาก /cmd_vel
        self.mode = 1

        rospy.Subscriber('/rb/cm/dr', String, self.callback_dr)
        rospy.Subscriber('/rb/cm/sv', String, self.callback_sv)
        rospy.Subscriber('/rb/cm/ed', String, self.callback_ed)
        rospy.Subscriber('/cmd_vel', Twist, self.callback_move_base)

        rospy.loginfo("Node command_handler initialized and running...")

    def process_message(self, msg, pub, data_type, max_value, topic_name):
        try:
            if not msg.data.strip():
                rospy.logwarn(f"Empty message received on {topic_name}")
                return
            data_int = int(msg.data, 16)
            if 0 <= data_int <= max_value:
                pub.publish(data_type(data_int))
                rospy.loginfo(f"Published {data_int} to {topic_name}")
            else:
                rospy.logwarn(f"Value out of range for {topic_name}: {data_int}")
        except ValueError:
            rospy.logerr(f"Invalid data received on {topic_name}: {msg.data} (cannot convert to integer)")

    def callback_move_base(self, msg):
        if self.mode == 0:
            self.auto_drive_data = UInt16(self.convert_cmd_vel_to_motor_data(msg))
            self.pub_motor.publish(self.auto_drive_data)
            rospy.loginfo(f"AUTO: Published to /arduino/drive: {self.auto_drive_data.data}")

    def callback_dr(self, msg):
        if self.mode == 1:
            try:
                self.manual_drive_data = UInt16(int(msg.data, 16))
                self.pub_motor.publish(self.manual_drive_data)
                rospy.loginfo(f"MANUAL: Published to /arduino/drive: {self.manual_drive_data.data}")
            except ValueError:
                rospy.logerr(f"Invalid data received on /rb/cm/dr: {msg.data}")

    def callback_sv(self, msg):
        self.process_message(msg, self.pub_servo, UInt8, 0xFF, '/arduino/servo')

    def callback_ed(self, msg):
        try:
            data = int(msg.data)
            value = data & 0xFFFFFF
            if (data >> 24) & 0xFF == 0x05:
                if value == 0:
                    self.mode = 0
                    rospy.loginfo("Switched to AUTO mode via /rb/cm/ed")
                    if self.auto_drive_data:
                        self.pub_motor.publish(self.auto_drive_data)
                elif value == 1:
                    self.mode = 1
                    rospy.loginfo("Switched to MANUAL mode via /rb/cm/ed")
                    if self.manual_drive_data:
                        self.pub_motor.publish(self.manual_drive_data)
                else:
                    rospy.logwarn(f"Invalid mode value received on /rb/cm/ed: {value}")
            else:
                self.process_message(msg, self.pub_edit, UInt32, 0xFFFFFFFF, '/arduino/edit')
        except (ValueError, AttributeError):
            rospy.logerr(f"Invalid data received on /rb/cm/ed: {msg.data}")


    def convert_cmd_vel_to_motor_data(self, msg):
        if not (-1.0 <= linear_x <= 1.0 and -1.0 <= angular_z <= 1.0):
            rospy.logwarn("Received /cmd_vel values out of range!")
            return 0x0000  # หยุดมอเตอร์

        linear_x = msg.linear.x
        angular_z = msg.angular.z
        pwm = abs(linear_x) * 255
        if linear_x > 0:
            return 0x0100 | int(pwm)
        elif linear_x < 0:
            return 0x0400 | int(pwm)
        elif angular_z > 0:
            return 0x0500 | int(pwm)
        elif angular_z < 0:
            return 0x0600 | int(pwm)
        else:
            return 0x0000
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        command_node = CommandHandler()
        command_node.run()
    except rospy.ROSInterruptException:
        pass
