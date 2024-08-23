from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node


class ControlServices(Node):

    def __init__(self):
        super().__init__('control_services')
        self.publisher_ = self.create_publisher(Twist, 'crazyflie/cmd_vel', 10)
        self.subscriber = self.create_subscription(Odometry, 'crazyflie/odometry', self.odometry_callback, 10)
        self.subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.takeoff_command = False
        self.current_pose = Odometry().pose.pose
        self.takeoff_height = 0.5
        self.max_ang_z_rate = 0.4
        self.is_flying = False
        self.keep_height = False
        self.teleop_cmd = Twist()

    def timer_callback(self):
        msg = self.teleop_cmd
        height_command = msg.linear.z

        new_cmd_msg = Twist()

        # If the drone is flying, only allow to transfer the twist message
        if self.is_flying:
            new_cmd_msg.linear.x = msg.linear.x
            new_cmd_msg.linear.y = msg.linear.y
            new_cmd_msg.linear.z = msg.linear.z
            new_cmd_msg.angular.x = msg.angular.x
            new_cmd_msg.angular.y = msg.angular.y
            new_cmd_msg.angular.z = msg.angular.z

        if height_command > 0 and not self.is_flying:
            new_cmd_msg.linear.z = 0.5
            if self.current_pose.position.z > self.takeoff_height:
                # stop going up if height is reached
                new_cmd_msg.linear.z = 0.0
                self.teleop_cmd.linear.z = 0.0
                self.is_flying = True
                self.get_logger().info('Takeoff completed')


        if height_command < 0 and self.is_flying:
            if self.current_pose.position.z < 0.1:
                new_cmd_msg.linear.z = 0.0
                self.is_flying = False
                self.keep_height = False
                self.get_logger().info('Landing completed')

        if abs(msg.angular.z) > self.max_ang_z_rate:
            new_cmd_msg.angular.z = self.max_ang_z_rate * abs(msg.angular.z)/msg.angular.z

        tolerance = 1e-7
        if abs(height_command) < tolerance and self.is_flying:
            if not self.keep_height:
                self.desired_height = self.current_pose.position.z
                self.keep_height = True
            else:
                error = self.desired_height - self.current_pose.position.z
                new_cmd_msg.linear.z = error

        if abs(height_command) > tolerance and self.is_flying:
            if self.keep_height:
                self.keep_height = False

        self.publisher_.publish(new_cmd_msg)

    def odometry_callback(self, msg):
        self.current_pose = msg.pose.pose

    def takeoff_callback(self, request, response):

        self.takeoff_command = True
        response.success = True
        return response

    def cmd_vel_callback(self, msg):
        self.teleop_cmd = msg


def main(args=None):
    rclpy.init(args=args)

    control_services = ControlServices()

    rclpy.spin(control_services)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
