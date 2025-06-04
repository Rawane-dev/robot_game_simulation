#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState

class FetchRotator:
    def __init__(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def get_robot_pose(self):
        response = self.get_model_state('fetch', 'world')
        return response.pose.position, response.pose.orientation

    def quaternion_to_yaw(self, orientation):
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        return yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def rotate_to_face_point(self, target_x, target_y):
        position, orientation = self.get_robot_pose()
        robot_x = position.x
        robot_y = position.y
        desired_yaw = math.atan2(target_y - robot_y, target_x - robot_x)
        current_yaw = self.quaternion_to_yaw(orientation)

        rospy.loginfo("Rotating to face target at (%.2f, %.2f)" % (target_x, target_y))

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.5
        tolerance = 0.01

        while not rospy.is_shutdown():
            _, orientation = self.get_robot_pose()
            current_yaw = self.quaternion_to_yaw(orientation)
            error = self.normalize_angle(desired_yaw - current_yaw)

            if abs(error) < tolerance:
                break

            twist.angular.z = 0.3 * error
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Finished rotation.")
