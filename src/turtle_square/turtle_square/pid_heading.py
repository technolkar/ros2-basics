import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class PID:
    """Reusable PID controller. You'll reuse this class on the real robot's motors."""

    def __init__(self, kp, ki, kd, out_limit, i_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.i_limit = i_limit          # anti-windup: cap the integral
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, error, dt):
        # I: accumulate error over time (clamped so it can't grow forever)
        self.integral += error * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))

        # D: rate of change of the error
        derivative = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error

        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.out_limit, min(self.out_limit, out))


class PIDHeading(Node):
    def __init__(self):
        super().__init__('pid_heading')

        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('disturbance', 0.0)   # simulated weak motor, rad/s
        p = lambda n: self.get_parameter(n).value
        self.pid = PID(p('kp'), p('ki'), p('kd'), out_limit=4.0, i_limit=2.0)
        self.disturbance = p('disturbance')

        # Step targets: 0°, 90°, 180°, -90°, changing every 5 s
        self.targets = [0.0, math.pi / 2, math.pi, -math.pi / 2]
        self.t_idx = 0
        self.dt = 0.05
        self.pose = None

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.target_pub = self.create_publisher(Float64, '/heading_target', 10)
        self.error_pub = self.create_publisher(Float64, '/heading_error', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.create_timer(self.dt, self.loop)
        self.create_timer(5.0, self.next_target)

        self.get_logger().info(
            f"PID kp={p('kp')} ki={p('ki')} kd={p('kd')} disturbance={self.disturbance}")

    def pose_cb(self, msg):
        self.pose = msg

    def next_target(self):
        self.t_idx = (self.t_idx + 1) % len(self.targets)
        self.pid.reset()
        self.get_logger().info(f'New target: {math.degrees(self.targets[self.t_idx]):.0f}°')

    def loop(self):
        if self.pose is None:
            return
        target = self.targets[self.t_idx]
        error = normalize_angle(target - self.pose.theta)

        cmd = Twist()
        cmd.angular.z = self.pid.update(error, self.dt) + self.disturbance
        self.cmd_pub.publish(cmd)

        self.target_pub.publish(Float64(data=target))
        self.error_pub.publish(Float64(data=error))


def main(args=None):
    rclpy.init(args=args)
    node = PIDHeading()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()