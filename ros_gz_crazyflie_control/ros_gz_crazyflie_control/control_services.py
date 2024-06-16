from std_srvs.srv import Trigger

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node


class ControlServices(Node):

    def __init__(self):
        super().__init__('control_services')
        self.srv = self.create_service(Trigger, 'crazyflie/takeoff', self.takeoff_callback)
        self.publisher_ = self.create_publisher(Twist, 'crazyflie/cmd_vel', 10)
        self.subscriber = self.create_subscription(Odometry, 'crazyflie/odometry', self.odometry_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.takeoff_command = False
        self.current_pose = Odometry().pose.pose
        self.takeoff_height = 0.5

    def timer_callback(self):
        if self.takeoff_command:
            msg = Twist()
            msg.linear.z = 0.5
            self.publisher_.publish(msg)
            if self.current_pose.position.z > self.takeoff_height:
                msg = Twist()
                self.publisher_.publish(msg)
                self.takeoff_command = False

    def odometry_callback(self, msg):
        self.current_pose = msg.pose.pose

    def takeoff_callback(self, request, response):

        self.takeoff_command = True
        return response


def main(args=None):
    rclpy.init(args=args)

    control_services = ControlServices()

    rclpy.spin(control_services)

    rclpy.shutdown()


if __name__ == '__main__':
    main()