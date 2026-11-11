"""SIMULATION ONLY: virtual VESC CAN messages -> simplified Gazebo Twist.

This is a command emulator, NOT actual VESC firmware, motor dynamics,
encoder feedback, or independent wheel torque simulation.
"""
import math
import time
import can
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from .protocol import decode_rpm, erpm_to_wheel_mps


class VirtualVescs(Node):
    def __init__(self):
        super().__init__('virtual_vescs')
        self.declare_parameter('interface', 'vcan0')
        self.declare_parameter('wheel_radius', 0.16)
        self.declare_parameter('wheel_separation', 0.68)
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('pole_pairs', 7)
        self.declare_parameter('timeout', 0.30)
        g = lambda n: self.get_parameter(n).value
        self.radius = float(g('wheel_radius'))
        self.track = float(g('wheel_separation'))
        self.ratio = float(g('gear_ratio'))
        self.poles = int(g('pole_pairs'))
        self.timeout = float(g('timeout'))
        if not all(math.isfinite(x) and x > 0 for x in (self.radius, self.track, self.ratio, self.poles, self.timeout)):
            raise ValueError('Invalid emulator parameters')
        interface = str(g('interface'))
        if not interface.startswith('vcan'):
            raise RuntimeError('SIM ONLY. Refusing physical interface '+interface)
        self.bus = can.Bus(channel=interface, interface='socketcan')
        self.last = {i: 0.0 for i in range(1,5)}
        self.erpm = {i: 0 for i in range(1,5)}
        self.pub = self.create_publisher(Twist, '/jeepbot/cmd_vel', 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info('Virtual VESC IDs 1,2,3,4 active')

    def tick(self):
        # Nonblocking drain: no extra reader thread is required.
        for _ in range(100):
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                break
            if not msg.is_extended_id:
                continue
            result = decode_rpm(msg.arbitration_id, bytes(msg.data))
            if result and result[0] in self.erpm:
                i, erpm = result
                self.erpm[i] = erpm
                self.last[i] = time.monotonic()
        now = time.monotonic()
        wheel = {}
        for i in range(1, 5):
            rpm = self.erpm[i] if now - self.last[i] <= self.timeout else 0
            wheel[i] = erpm_to_wheel_mps(rpm, self.radius, self.ratio, self.poles)
        left = (wheel[1] + wheel[3]) / 2
        right = (wheel[2] + wheel[4]) / 2
        twist = Twist()
        twist.linear.x = (left + right) / 2
        twist.angular.z = (right - left) / self.track
        self.pub.publish(twist)

    def close(self):
        self.bus.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = VirtualVescs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
