"""SIMULATION ONLY: /cmd_vel -> 4 VESC RPM frames on vcan0."""
import math
import time
import can
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from .protocol import encode_rpm, wheel_mps_to_erpm, WHEEL_IDS


class SkidController(Node):
    def __init__(self):
        super().__init__('sim_skid_controller')
        self.declare_parameter('interface', 'vcan0')
        self.declare_parameter('wheel_radius', 0.16)
        self.declare_parameter('wheel_separation', 0.68)
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('pole_pairs', 7)
        self.declare_parameter('max_wheel_speed', 0.5)
        self.declare_parameter('command_timeout', 0.25)
        get = lambda key: self.get_parameter(key).value
        self.radius = float(get('wheel_radius'))
        self.track = float(get('wheel_separation'))
        self.ratio = float(get('gear_ratio'))
        self.poles = int(get('pole_pairs'))
        self.max_speed = float(get('max_wheel_speed'))
        self.timeout = float(get('command_timeout'))
        if not all(x > 0 and math.isfinite(x) for x in (self.radius, self.track, self.ratio, self.poles, self.max_speed, self.timeout)):
            raise ValueError('Simulation parameters must be positive')
        interface = str(get('interface'))
        if not interface.startswith('vcan'):
            raise RuntimeError('SIM ONLY. Refusing a physical CAN interface: '+interface)
        self.bus = can.Bus(channel=interface, interface='socketcan')
        self.target_v = self.target_w = 0.0
        self.last_cmd = time.monotonic()
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_timer(0.05, self.on_timer)
        self.get_logger().info('Publishing four virtual VESC RPM commands via '+interface)

    def on_cmd(self, msg):
        v, w = float(msg.linear.x), float(msg.angular.z)
        if not (math.isfinite(v) and math.isfinite(w)):
            return
        self.target_v, self.target_w = v, w
        self.last_cmd = time.monotonic()

    def send_wheels(self, left, right):
        for wheel, velocity in [('FL', left), ('FR', right), ('RL', left), ('RR', right)]:
            erpm = wheel_mps_to_erpm(velocity, self.radius, self.ratio, self.poles)
            frame_id, data = encode_rpm(WHEEL_IDS[wheel], erpm)
            self.bus.send(can.Message(arbitration_id=frame_id, is_extended_id=True, data=data), timeout=0.02)

    def on_timer(self):
        v, w = (0.0, 0.0) if time.monotonic() - self.last_cmd > self.timeout else (self.target_v, self.target_w)
        left, right = v - w * self.track / 2, v + w * self.track / 2
        peak = max(abs(left), abs(right))
        if peak > self.max_speed:
            scale = self.max_speed / peak
            left, right = left * scale, right * scale
        try:
            self.send_wheels(left, right)
        except can.CanError as exc:
            self.get_logger().error('Virtual CAN error: '+str(exc))

    def close(self):
        try:
            self.send_wheels(0.0, 0.0)
        except can.CanError:
            pass
        self.bus.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SkidController()
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
