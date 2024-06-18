#!/usr/bin/env python3

""" This simple mapper is loosely based on both the bitcraze cflib point cloud example 
 https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/multiranger/multiranger_pointcloud.py
 and the webots epuck simple mapper example:
 https://github.com/cyberbotics/webots_ros2

 Originally from https://github.com/knmcguire/crazyflie_ros2_experimental/
 """

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from tf2_ros import StaticTransformBroadcaster

import tf_transformations
import math
import numpy as np
from .wall_following.wall_following import WallFollowing
import time

GLOBAL_SIZE_X = 20.0
GLOBAL_SIZE_Y = 20.0
MAP_RES = 0.1


class WallFollowingMultiranger(Node):
    def __init__(self):
        super().__init__('simple_mapper_multiranger')
        self.declare_parameter('robot_prefix', '/crazyflie')
        robot_prefix = self.get_parameter('robot_prefix').value

        self.odom_subscriber = self.create_subscription(
            Odometry, robot_prefix + '/odometry', self.odom_subscribe_callback, 10)
        self.ranges_subscriber = self.create_subscription(
            LaserScan, '/lidar', self.scan_subscribe_callback, 10)
        self.position = [0.0, 0.0, 0.0]
        self.angles = [0.0, 0.0, 0.0]
        self.ranges = [0.0, 0.0, 0.0, 0.0]

        self.position_update = False

        self.twist_publisher = self.create_publisher(Twist, robot_prefix + '/cmd_vel', 10)

        self.get_logger().info(f"Wall following set for crazyflie " + robot_prefix +
                               f" using the scan topic")
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.wall_following = WallFollowing(
        angle_value_buffer=0.05, reference_distance_from_wall=1.0,
        max_forward_speed=0.5, init_state=WallFollowing.StateWallFollowing.FORWARD)

    def timer_callback(self):
        # initialize variables
        velocity_x = 0.0
        velocity_y = 0.0
        yaw_rate = 0.0
        state_wf = WallFollowing.StateWallFollowing.HOVER

        # Get Yaw
        actual_yaw_rad = self.angles[2]

        # get front range in meters
        back_range = self.ranges[0]
        right_range = self.ranges[1]
        front_range = self.ranges[2]
        left_range = self.ranges[3]

        self.get_logger().info(f"Front range: {front_range}, Right range: {right_range}, Left range: {left_range}")

        # choose here the direction that you want the wall following to turn to
        wall_following_direction = WallFollowing.WallFollowingDirection.RIGHT
        side_range = left_range

        # get velocity commands and current state from wall following state machine
        if side_range > 0.1:
            velocity_x, velocity_y, yaw_rate, state_wf = self.wall_following.wall_follower(
                front_range, side_range, actual_yaw_rad, wall_following_direction, time.time())
        

        msg = Twist()
        msg.linear.x = velocity_x
        msg.linear.y = velocity_y
        msg.angular.z = yaw_rate
        self.twist_publisher.publish(msg)

        print('velocity_x', velocity_x, 'velocity_y', velocity_y,
                'yaw_rate', yaw_rate, 'state_wf', state_wf)


    def odom_subscribe_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angles[0] = euler[0]
        self.angles[1] = euler[1]
        self.angles[2] = euler[2]
        self.position_update = True

    def scan_subscribe_callback(self, msg):
        self.ranges = msg.ranges

def main(args=None):

    rclpy.init(args=args)
    wall_following_multiranger = WallFollowingMultiranger()
    rclpy.spin(wall_following_multiranger)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
