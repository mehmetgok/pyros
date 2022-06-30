

# $ roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
# $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

from math import pi

import rospy
import tf.transformations

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

# TB3 icin
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist

class PoseListener:
    def __init__(self):
        self.counter = 0
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

        # Normal Robot Icin
        # rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.pose_callback)

        # Turtlebot3 icin
        rospy.Subscriber("/odom", Odometry, self.pose_callback)

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(10)      

    def goal_callback(self, msg):
        # Copying for simplicity
        position = msg.pose.position
        quat = msg.pose.orientation
        rospy.loginfo("Point Position: [ %f, %f, %f ]" % (position.x, position.y, position.z))
        rospy.loginfo("Quat Orientation: [ %f, %f, %f, %f]" % (quat.x, quat.y, quat.z, quat.w))

        # Also print Roll, Pitch, Yaw
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        rospy.loginfo("Euler Angles: %s" % str((euler[2]*180)/pi))  

        self.turn2goal(euler[2])
    
    def pose_callback(self, msg):
        position = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        self.x = position.x
        self.y = position.y

        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        self.theta = euler[2]

        # print(self.theta)

        # print(self.x, self.y)
    
    def turn2goal(self, goal_angle):
        """Turns to the goal"""
        vel_msg = Twist()

        while True:
            # Linear velocity in the x-axis.
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0.59*(goal_angle-self.theta)

            # print("Difference: {}".format(vel_msg.angular.z ))

            if abs(vel_msg.angular.z)<0.02:
                break
            
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        print("Turn 2 goal completed")
        self.velocity_publisher.publish(Twist())


if __name__ == '__main__':
    rospy.init_node('number_counter')
    PoseListener()
    rospy.spin()