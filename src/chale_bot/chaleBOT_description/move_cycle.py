#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MoveCycleNode(Node):
    def __init__(self):
        super().__init__('move_cycle_node')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # timings
        self.move_duration = 10.0
        self.stop_duration = 10.0

        # slow speed
        self.linear_speed = 2.0   # m/s
        self.angular_speed = 0

        # state tracking
        self.is_moving = True
        self.state_start_time = self.get_clock().now()

        # timer runs every 0.1 sec
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Move cycle node started.')
        self.get_logger().info('Robot will move for 10 s, stop for 4 s, and repeat.')

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds / 1e9

        msg = Twist()

        if self.is_moving:
            if elapsed < self.move_duration:
                msg.linear.x = self.linear_speed
                msg.angular.z = self.angular_speed
            else:
                self.is_moving = False
                self.state_start_time = now
                self.get_logger().info('Stopping for 4 seconds.')
                msg.linear.x = 0.0
                msg.angular.z = 0.0
        else:
            if elapsed < self.stop_duration:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            else:
                self.is_moving = True
                self.state_start_time = now
                self.get_logger().info('Moving again for 10 seconds.')
                msg.linear.x = self.linear_speed
                msg.angular.z = self.angular_speed

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MoveCycleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # stop robot before shutdown
    stop_msg = Twist()
    node.publisher_.publish(stop_msg)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
