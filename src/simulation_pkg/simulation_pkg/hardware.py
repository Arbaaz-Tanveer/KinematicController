# vision_pkg/hardware.py
import math
import threading
import time
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class OmniKinematics:
    def __init__(self, wheel_radius: float, bot_radius: float):
        self.wheel_radius = wheel_radius
        self.bot_radius   = bot_radius

    def compute_motion(self, d1, d2, d3, d4, dt):
        w1 = d1 * self.wheel_radius
        w2 = d2 * self.wheel_radius
        w3 = d3 * self.wheel_radius
        w4 = d4 * self.wheel_radius

        dx = (w1 + w2 + w3 + w4) * math.sqrt(2) / 4.0
        dy = (-w1 + w2 + w3 - w4) * math.sqrt(2) / 4.0
        dth = (-w1 - w2 + w3 + w4) / (4.0 * self.bot_radius)
        return dx, dy, dth

    def compute_wheel_velocities(self, vx, vy, omega):
        s = 1.0 / math.sqrt(2)
        v1 = ((vx - vy) - omega * self.bot_radius) / self.wheel_radius
        v2 = ((vx + vy) - omega * self.bot_radius) / self.wheel_radius
        v3 = ((vx + vy) + omega * self.bot_radius) / self.wheel_radius
        v4 = ((vx - vy) + omega * self.bot_radius) / self.wheel_radius
        return [v1, v2, v3, v4]

class OmniSerialNode(Node):
    def __init__(self):
        super().__init__('hardware')
        # parameters
        self.declare_parameter('wheel_radius', 0.05 * 6.28 / 1800)
        self.declare_parameter('bot_radius',   0.25)
        self.declare_parameter('port',         '/dev/ttyACM0')
        self.declare_parameter('baudrate',     9600)

        wr = self.get_parameter('wheel_radius').value
        br = self.get_parameter('bot_radius').value
        port   = self.get_parameter('port').value
        baud   = self.get_parameter('baudrate').value

        # kinematics, serial, buffer
        self.kin = OmniKinematics(wr, br)
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2)
    

        self.prev_enc = None
        self.prev_time = None

        # state + threading lock
        self.vx = self.vy = self.omega = 0.0
        self.lock = threading.Lock()

        # ROS topics
        self.create_subscription(Twist, '/cmd_vel_local', self.cmd_vel_cb, 10)
        self.odom_pub = self.create_publisher(Float32MultiArray, 'odom_delta', 10)

        # timer to spin serial at up to 1 kHz
        self.create_timer(0.03, self.spin_once)

    def cmd_vel_cb(self, msg: Twist):
        with self.lock:
            self.vx    = msg.linear.x
            self.vy    = msg.linear.y
            self.omega = msg.angular.z

    def spin_once(self):
        line = None
        if self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').strip()

        if line:
            parts = line.split(':')
            if len(parts) >= 5:
                try:
                    enc = list(map(int, parts[:4]))
                    ts  = float(parts[4])
                    if self.prev_enc is None:
                        self.prev_enc = enc
                        self.prev_time = ts
                    else:
                        d_enc = [enc[i] - self.prev_enc[i] for i in range(4)]
                        dt = (ts - self.prev_time) / 1000.0
                        if dt > 0:
                            dx, dy, dth = self.kin.compute_motion(*d_enc, dt)
                            msg = Float32MultiArray()
                            msg.data = [ts, dx, dy, dth]
                            self.odom_pub.publish(msg)
                        

                        self.prev_enc = enc
                        self.prev_time = ts
                except ValueError:
                    self.get_logger().warn(f'Bad line: {line}')

        # send velocity
        with self.lock:
            wheels = self.kin.compute_wheel_velocities(self.vx, self.vy, self.omega)
        signs = ['0' if v >= 0 else '1' for v in wheels]
        mags  = [abs(round(v)) for v in wheels]
        bits  = ''.join(signs)
        out   = f'{mags[0]},{mags[1]},{mags[2]},{mags[3]},{bits}\n'
        print(out)
        self.ser.write(out.encode())

def main(args=None):
    rclpy.init(args=args)
    node = OmniSerialNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()