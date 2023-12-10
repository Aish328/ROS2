#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

class SnowmanController(Node):
    def __init__(self):
        super().__init__('snowman_controller')
        self.subscription1 = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback1,
            10
        )
        self.publisher1 = self.create_publisher(
            Twist,
            'turtle1/cmd_vel',
            10
        )
        self.radius_small = 2.0
        self.radius_big = 4.0
        self.is_first_turtle_done = False
        self.angle1 = 0.0

    def pose_callback1(self, msg):
        if not self.is_first_turtle_done:
            if self.angle1 < 2 * 3.14:
                # Calculate linear and angular velocities to create a circle (smaller)
                linear_speed = self.radius_small
                angular_speed = 1.0
            else:
                # Stop the first turtle when the smaller circle is complete
                linear_speed = 0.0
                angular_speed = 0.0
                self.is_first_turtle_done = True
                self.spawn_second_turtle()

            # Publish velocity commands to the first turtle
            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed
            self.publisher1.publish(twist)
            self.angle1 += 0.02

    def spawn_second_turtle(self):
        spawn_client = self.create_client(Spawn, 'spawn')
        while not spawn_client.service_is_ready():
            self.get_logger().info('Service /spawn not available, waiting again...')

        request = Spawn.Request()
        request.name = 'turtle2'
        request.x = 5.0  # Adjust the initial x position as needed
        request.y = 5.0  # Adjust the initial y position as needed
        request.theta = 0.0

        future = spawn_client.call(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Spawned turtle2 successfully')
            self.create_second_turtle()
        else:
            self.get_logger().warning('Failed to spawn turtle2')


    def create_second_turtle(self):
        self.subscription2 = self.create_subscription(
            Pose,
            'turtle2/pose',
            self.pose_callback2,
            10
        )
        self.publisher2 = self.create_publisher(
            Twist,
            'turtle2/cmd_vel',
            10
        )
        self.angle2 = 0.0

    def pose_callback2(self, msg):
        if self.is_first_turtle_done:
            if self.angle2 < 2 * 3.14:
                # Calculate linear and angular velocities to create a circle (bigger)
                linear_speed = self.radius_big
                angular_speed = 0.5  # Smaller angular speed for a bigger circle
            else:
                # Stop the second turtle when the bigger circle is complete
                linear_speed = 0.0
                angular_speed = 0.0
            # Publish velocity commands to the second turtle
            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed
            self.publisher2.publish(twist)
            self.angle2 += 0.02

def main(args=None):
    rclpy.init(args=args)
    snowman_controller = SnowmanController()
    rclpy.spin(snowman_controller)
    snowman_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
