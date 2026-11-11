import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SquareDriver(Node):
    def __init__(self):
        super().__init__('square_driver')                 # node name

        # Publisher: sends Twist messages on the turtle's velocity topic
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Parameter: can be changed from the command line without editing code
        self.declare_parameter('side_time', 2.0)
        self.side_time = self.get_parameter('side_time').value

        self.linear_speed = 1.0                            # m/s
        self.angular_speed = 1.0                           # rad/s
        self.turn_time = (math.pi / 2) / self.angular_speed   # time for 90°

        self.state = 'forward'
        self.state_start = self.get_clock().now()

        # Timer: calls self.loop every 0.1 s (10 Hz)
        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info(
            f'Square driver started: side={self.side_time * self.linear_speed:.1f} m')

    def loop(self):
        elapsed = (self.get_clock().now() - self.state_start).nanoseconds / 1e9
        msg = Twist()

        if self.state == 'forward':
            msg.linear.x = self.linear_speed
            if elapsed >= self.side_time:
                self.switch('turn')
        else:  # turning
            msg.angular.z = self.angular_speed
            if elapsed >= self.turn_time:
                self.switch('forward')

        self.pub.publish(msg)

    def switch(self, new_state):
        self.state = new_state
        self.state_start = self.get_clock().now()
        self.get_logger().info(f'-> {new_state}')


def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
    try:
        rclpy.spin(node)          # keep running, calling the timer
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()