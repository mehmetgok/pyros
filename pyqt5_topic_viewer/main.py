# pytqt5_encoders
# Python3 Qt5 "ROS Nooetic" Application to show info from 2WD Rover
# 2WD Rover Firmware uses Linobot Derivative

import rospy

from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QApplication
import sys

from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Float32

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

from math import pi



class Ui(QtWidgets.QMainWindow):
    def __init__(self):
        super(Ui, self).__init__()
        uic.loadUi('mainwnd.ui', self)

        # Find the button
        self.button = self.findChild(QtWidgets.QPushButton, 'pbReset')      
        self.button.clicked.connect(self.resetButtonPressed) 

        rospy.Subscriber('left_ticks', Int32, self.display_left_ticks)
        rospy.Subscriber('right_ticks', Int32, self.display_right_ticks)

        rospy.Subscriber('left_rpm', Int16, self.display_left_rpm)
        rospy.Subscriber('right_rpm', Int16, self.display_right_rpm)

        rospy.Subscriber('left_pwm', Int16, self.display_left_pwm)
        rospy.Subscriber('right_pwm', Int16, self.display_right_pwm)

        rospy.Subscriber('linear_speed', Float32, self.display_linear_speed)
        rospy.Subscriber('angular_speed', Float32, self.display_angular_speed)

        rospy.Subscriber('current_data', Float32, self.display_current)
        rospy.Subscriber('voltage_data', Float32, self.display_voltage)

        
        self.odom_sub = rospy.Subscriber('robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.display_odom)

        self.show()

    def resetButtonPressed(self):
        print("Reset Button Pressed")
    
    def display_left_ticks(self, data):
        self.lcd_left.display(data.data)
        # print(data.data)        

    def display_right_ticks(self, data):
        self.lcd_right.display(data.data)
        # print(data.data)
    
    def display_right_rpm(self, data):
        self.lcd_right_rpm.display(data.data)
        # print(data.data)
    
    def display_left_rpm(self, data):
        self.lcd_left_rpm.display(data.data)
        # print(data.data)
    
    def display_right_pwm(self, data):
        self.lcd_right_pwm.display(data.data)
        # print(data.data)
    
    def display_left_pwm(self, data):
        self.lcd_left_pwm.display(data.data)
        # print(data.data)
    
    def display_linear_speed(self, data):
        self.lcd_linear_speed.display(data.data)
        # print(data.data)
    
    def display_angular_speed(self, data):
        self.lcd_angular_speed.display(data.data)
        # print(data.data)
    
    def display_current(self, data):
        self.lcd_current.display(data.data*1000.0)
        # print(data.data)
    
    def display_voltage(self, data):
        self.lcd_voltage.display(data.data)
        # print(data.data)
    
    def display_odom(self, data):
        # self.lcd_voltage.display(data.data)
        # print(data.data)
        # print(data.pose.pose.position.x)
        # print(data.pose.pose.position.y)
        # print(data.pose.pose.orientation.w)
        self.lcd_x.display(data.pose.pose.position.x)
        self.lcd_y.display(data.pose.pose.position.y)
        # self.lcd_theta.display(data.pose.pose.orientation.w)
        rot_q = data.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        theta_deg = (theta) / (2 * pi)  * 360.0
        if theta_deg<0: theta_deg += 360.0
        self.lcd_theta.display(theta_deg)
        
        

if __name__ == "__main__":
    rospy.init_node('qt_encoder_viewer')
    app = QApplication(sys.argv)
    window = Ui()
    exit(app.exec_())