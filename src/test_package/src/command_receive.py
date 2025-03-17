#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, UInt32, UInt16, UInt8
from geometry_msgs.msg import Twist

class CommandHandler:
    def __init__(self):
        rospy.init_node('command_node', anonymous=True)
        #self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_motor_command)  # 50ms #Timeout prevent command 
        self.pub_motor = rospy.Publisher('/arduino/drive', UInt16, queue_size=100)
        self.pub_servo = rospy.Publisher('/arduino/servo', UInt8, queue_size=100)
        self.pub_edit = rospy.Publisher('/arduino/edit', UInt32, queue_size=100)
        self.manual_drive_data = None  # เก็บคำสั่งจาก /rb/cm/dr
        self.auto_drive_data = None    # เก็บคำสั่งจาก /cmd_vel
        self.mode = 1
        rospy.loginfo(f"Initial Mode: {'MANUAL' if self.mode == 1 else 'AUTO'}")


        rospy.Subscriber('/rb/cm/dr', UInt16, self.callback_dr)
        rospy.Subscriber('/rb/cm/sv', UInt16, self.callback_sv)
        rospy.Subscriber('/rb/cm/ed', UInt32, self.callback_ed)
        rospy.Subscriber('/cmd_vel', Twist, self.callback_move_base)

        rospy.loginfo("Node command_handler initialized and running...")

    #Timeout prevent command 
    """
    def publish_motor_command(self, event):
        if self.mode == 0 and self.auto_drive_data is not None:
            self.pub_motor.publish(self.auto_drive_data)
    """

    def process_message(self, msg, pub, max_value, topic_name):
        try:
            if not isinstance(msg.data, int):
                rospy.logerr(f"Invalid data type received on {topic_name}: {msg.data}")
                return
            data_int = msg.data
            if 0 <= data_int <= max_value:
                pub.publish(data_int)
                rospy.loginfo(f"Published {data_int} to {topic_name}")
            else:
                rospy.logwarn(f"Value out of range for {topic_name}: {data_int}")
        except ValueError:
            rospy.logerr(f"Invalid data received on {topic_name}: {msg.data} ")

    def callback_move_base(self, msg):
        if self.mode == 0:
            new_auto_drive_data = self.convert_cmd_vel_to_motor_data(msg)
            if self.auto_drive_data is None or new_auto_drive_data.data != self.auto_drive_data.data:
                self.auto_drive_data = new_auto_drive_data
                self.pub_motor.publish(self.auto_drive_data)
                rospy.loginfo(f"AUTO: Published to /arduino/drive: {self.auto_drive_data.data}")

    def callback_dr(self, msg):
        if self.mode == 1:
            try:
                self.manual_drive_data = msg.data
                self.pub_motor.publish(self.manual_drive_data)
                rospy.loginfo(f"MANUAL: Published to /arduino/drive: {self.manual_drive_data}")
            except ValueError:
                rospy.logerr(f"Invalid data received on /rb/cm/dr: {msg.data}")

    def callback_sv(self, msg):
        self.process_message(msg, self.pub_servo, 0xFF, '/arduino/servo')

    def callback_ed(self, msg):
        try:
            data = int(msg.data)
            value = data & 0xFFFFFF
            if (data >> 24) & 0xFF == 0x05:
                if value == 0: #AUTO
                    self.mode = 0 #set mode
                    self.manual_drive_data = None  #clear command
                    rospy.loginfo("Switched to AUTO mode via /rb/cm/ed")
                    if self.auto_drive_data:
                        self.pub_motor.publish(self.auto_drive_data)
                elif value == 1: #MANUAL
                    self.mode = 1  #set mode
                    self.auto_drive_data = None #clear command
                    rospy.loginfo("Switched to MANUAL mode via /rb/cm/ed")
                    if self.manual_drive_data:
                        self.pub_motor.publish(self.manual_drive_data)
                else:
                    rospy.logwarn(f"Invalid mode value received on /rb/cm/ed: {value}")
            else:
                self.process_message(msg, self.pub_edit, 0xFFFFFFFF, '/arduino/edit')
        except (ValueError, AttributeError):
            rospy.logerr(f"Invalid data received on /rb/cm/ed: {msg.data}")

    def convert_cmd_vel_to_motor_data(self, msg):
        linear_x = msg.linear.x  # เดินหน้า-ถอยหลัง
        linear_y = msg.linear.y  # เลื่อนไปทางซ้าย-ขวา (Strafing)
        angular_z = msg.angular.z  # หมุนรอบตัวเอง

        # ตรวจสอบว่าค่าที่ได้รับอยู่ในช่วงที่กำหนด
        if not (-1.0 <= linear_x <= 1.0 and -1.0 <= linear_y <= 1.0 and -1.0 <= angular_z <= 1.0):
            rospy.logwarn("Received /cmd_vel values out of range!")
            return UInt16(0x0000)  # หยุดมอเตอร์

        pwm_x = int(abs(linear_x) * 255)
        pwm_y = int(abs(linear_y) * 255)
        pwm_z = int(abs(angular_z) * 255)

        command = 0x0000

        # ใช้ OR เพื่อรวมทิศทางที่สามารถทำงานพร้อมกันได้
        if linear_x > 0:
            command |= 0x0100 | pwm_x  # ไปข้างหน้า
        elif linear_x < 0:
            command |= 0x0400 | pwm_x  # ถอยหลัง

        if linear_y > 0:
            command |= 0x0200 | pwm_y  # เลื่อนไปทางขวา
        elif linear_y < 0:
            command |= 0x0300 | pwm_y  # เลื่อนไปทางซ้าย

        if angular_z > 0:
            command |= 0x0500 | pwm_z  # หมุนตามเข็มนาฬิกา
        elif angular_z < 0:
            command |= 0x0600 | pwm_z  # หมุนทวนเข็มนาฬิกา

        return UInt16(command)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        command_node = CommandHandler()
        command_node.run()
    except rospy.ROSInterruptException:
        pass
