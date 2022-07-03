import sys

from math import pi
from time import sleep



from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist

import rospy

from PyQt5.QtWidgets import QProgressBar
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QThread
from PyQt5.QtCore import QWaitCondition
from PyQt5.QtCore import QMutex
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5 import QtGui, QtCore, QtWidgets

from PyQt5.QtCore import QTimer

from PyQt5 import uic


class Thread(QThread):
    change_pose = pyqtSignal(float, float, float)

    def __init__(self):
        QThread.__init__(self)
        self.cond = QWaitCondition()
        self.mutex = QMutex()
        self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)

        self.rate = rospy.Rate(10)

        

    def get_rotation(self, msg):

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
       
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
       

        self.change_pose.emit(x, y, yaw) 


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi("mainwnd.ui", self)

        self.pbTurn.clicked.connect(self.startTurn)

        self.th = Thread()
        self.th.change_pose.connect(self.getData)   
        self.th.start()

        self.timer = QTimer()

        self.timer.timeout.connect(self.turn)
        self.timer.setInterval(200)

        self.kp = 0.7

        self.yaw = 0.0

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.command = Twist()
    
    def startTurn(self):
        self.timer.start()
        self.pbTurn.setEnabled(False)

    
    def getData(self, x, y, theta):
        self.lcdX.display(x)
        self.lcdY.display(y)
        self.lcdTheta.display(theta*180/pi)
        self.yaw = theta

   
    def turn(self):

        target = self.leAngle.text()

        target = float(target)

        if target>180:
            target -= 360

        target_rad = target*pi/180
        self.command.angular.z = self.kp * (target_rad-self.yaw)
        self.pub.publish(self.command)
        print("taeget={} current:{}".format(target, self.yaw))

        if target_rad - self.yaw < 0.1:
            self.pub.publish(Twist())
            self.pbTurn.setEnabled(True)
            self.timer.stop()
        
        




   
        


if __name__ == "__main__":
    rospy.init_node('progress')
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    exit(app.exec_())


