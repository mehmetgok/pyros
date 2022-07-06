# $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
# $ roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch


# https://github.com/kulbir-ahluwalia/Turtlebot_3_PID/blob/master/control_bot/Scripts/final.py


from datetime import datetime
import rospy
from geometry_msgs.msg import Twist, Point
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
from tf.transformations import quaternion_from_euler

# Turtlebot3 icin
from geometry_msgs.msg import PoseStamped
# Kendi robotumuz icin
from geometry_msgs.msg import PoseWithCovarianceStamped

# TB3 icin
from nav_msgs.msg import Odometry

from std_msgs.msg import Float32


kp_distance = 1
ki_distance = 0.01
kd_distance = 0.5

kp_angle = 1
ki_angle = 0.03
kd_angle = 0.05

class GotoPoint():
    def __init__(self):
        rospy.init_node('turtlebot3_pointop_key', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        position = Point()
        
        self.rate = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'

        self.counter = 0

        self.log_file = open('emek_log.txt')
        
        # RViz den hedef koordinatı al
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)


        rospy.Subscriber('current_data', Float32, self.update_current)
        rospy.Subscriber('voltage_data', Float32, self.update_voltage)
    
        # Normal Robot Icin
        # rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.pose_callback)

        # Turtlebot3 icin
        # rospy.Subscriber("/odom", Odometry, self.pose_callback)

        self.current_data = 0.0
        self.voltage_data = 0.0


        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        (position, rotation) = self.get_odom()


        """
        (goal_x, goal_y, goal_z) = self.getkey()
        if goal_z > 180 or goal_z < -180:
            print("you input wrong z range.")
            self.shutdown()
        
        goal_z = np.deg2rad(goal_z)
        """
    
    def update_current(self, data):
        self.current_data = data.data*1000.0
      
    def update_voltage(self, data):
        self.voltage_data = data.data
  
    
    def go2goal(self):

        move_cmd = Twist()

        last_rotation = 0
        linear_speed = 1   # kp_distance
        angular_speed = 1  # kp_angular

        (position, rotation) = self.get_odom()

  



        goal_distance = sqrt(pow(self.goal_x - position.x, 2) + pow(self.goal_y - position.y, 2))
        #distance is the error for length, x,y
        distance = goal_distance
        previous_distance = 0
        total_distance = 0

        previous_angle = 0
        total_angle = 0
    
        while abs(rotation - self.goal_z) > 0.05:
            (position, rotation) = self.get_odom()
            if self.goal_z >= 0:
                if rotation <= self.goal_z and rotation >= self.goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= self.goal_z + pi and rotation > self.goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
            self.cmd_vel.publish(move_cmd)
            self.rate.sleep()




  
        while distance > 0.05:
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            #path_angle = error
            path_angle = atan2(self.goal_y - y_start, self.goal_x- x_start)

            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation

            diff_angle = path_angle - previous_angle
            diff_distance = distance - previous_distance

            distance = sqrt(pow((self.goal_x - x_start), 2) + pow((self.goal_y - y_start), 2))

            control_signal_distance = kp_distance*distance + ki_distance*total_distance + kd_distance*diff_distance

            control_signal_angle = kp_angle*path_angle + ki_angle*total_angle + kd_distance*diff_angle

            move_cmd.angular.z = (control_signal_angle) - rotation
            #move_cmd.linear.x = min(linear_speed * distance, 0.1)
            move_cmd.linear.x = min(control_signal_distance, 0.1)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            last_rotation = rotation
            self.cmd_vel.publish(move_cmd)
            self.rate.sleep()
            previous_distance = distance
            total_distance = total_distance + distance
            print("Current positin and rotation are: ", (position, rotation))

            now = datetime.now() # current date and time

            time_str = now.strftime("%H:%M:%S")


            self.f.write(time_str + ";" + str(float(position.x)) +";"+ str(float(position.y)) + ";" + 
                str(self.current_data) + ";" + str(self.voltage_data) + " ;" + str(move_cmd.linear.x) + ";"+ str(move_cmd.angular.z) + "\n")
        

        (position, rotation) = self.get_odom()
        print("Current positin and rotation are: ", (position, rotation))

        print("reached :)   ^_^")
         
        # rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())



    
    def goal_callback(self, msg):
        # Copying for simplicity
        position = msg.pose.position
        quat = msg.pose.orientation
        rospy.loginfo("Point Position: [ %f, %f, %f ]" % (position.x, position.y, position.z))
        rospy.loginfo("Quat Orientation: [ %f, %f, %f, %f]" % (quat.x, quat.y, quat.z, quat.w))

        # Also print Roll, Pitch, Yaw
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        rospy.loginfo("Euler Angles: %s" % str((euler[2]*180)/pi))
        
        self.goal_x = position.x
        self.goal_y = position.y
        self.goal_z = euler[2]
        
        self.go2goal()

        

        # self.turn2goal(euler[2])


    def getkey(self):
        global x_input, y_input, z_input
        x = x_input
        y = y_input
        z = z_input
        if x == 's':
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])


    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


# time.sleep(5)

"""
while not rospy.is_shutdown():
    GotoPoint()
"""


if __name__ == '__main__':
   
    GotoPoint()
    rospy.spin()