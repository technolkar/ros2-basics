import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


def normalize_angle(a):
    """Wrap an angle to [-pi, pi] so the turtle always turns the short way."""
    return math.atan2(math.sin(a), math.cos(a))


class ClosedLoopSquare(Node):
    def __init__(self):
        super().__init__('closed_loop_square')

        # Parameters (tunable from the command line)
        self.declare_parameter('side', 2.0)          # m
        self.declare_parameter('kp_linear', 1.5)
        self.declare_parameter('kp_angular', 6.0)
        self.side = self.get_parameter('side').value
        self.kp_lin = self.get_parameter('kp_linear').value
        self.kp_ang = self.get_parameter('kp_angular').value

        self.pose = None          # latest measured pose (feedback)
        self.waypoints = []       # 4 corners, fixed in the world
        self.idx = 0              # which corner we're driving to

        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)   # 20 Hz

    def pose_callback(self, msg):
        """Called every time turtlesim publishes a new pose."""
        self.pose = msg
        if not self.waypoints:    # first message: build the square from the start point
            x0, y0, s = msg.x, msg.y, self.side
            self.waypoints = [(x0 + s, y0), (x0 + s, y0 + s), (x0, y0 + s), (x0, y0)]
            self.get_logger().info(f'Square corners: {self.waypoints}')

    def control_loop(self):
        if self.pose is None:     # no feedback yet → do nothing
            return

        gx, gy = self.waypoints[self.idx]
        dx, dy = gx - self.pose.x, gy - self.pose.y
        distance_error = math.hypot(dx, dy)
        heading_error = normalize_angle(math.atan2(dy, dx) - self.pose.theta)

        cmd = Twist()
        if distance_error < 0.05:
            # Corner reached → next one
            self.get_logger().info(f'Reached corner {self.idx} (error {distance_error:.3f} m)')
            self.idx = (self.idx + 1) % 4
        elif abs(heading_error) > 0.05:
            # Not facing the corner yet → turn in place (P-control on angle)
            cmd.angular.z = max(-3.0, min(3.0, self.kp_ang * heading_error))
        else:
            # Facing the corner → drive (P-control on distance + small heading correction)
            cmd.linear.x = min(2.0, self.kp_lin * distance_error)
            cmd.angular.z = self.kp_ang * heading_error

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopSquare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()