"""Nav2/missionの排他的切替。全指令をCollision Monitorへ流す。"""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool


class CommandGate(Node):
    def __init__(self):
        super().__init__('box_command_gate')
        for name, default in {'command_timeout': 0.4, 'scan_timeout': 0.7,
                              'nav_topic': '/cmd_vel_nav_stack',
                              'mission_topic': '/box_mission/cmd_vel',
                              'output_topic': '/cmd_vel_raw', 'scan_topic': '/low_scan'}.items():
            self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value
        self.timeout, self.scan_timeout = p('command_timeout'), p('scan_timeout')
        self.owner = False
        self.commands = {'nav': (Twist(), 0.), 'mission': (Twist(), 0.)}
        self.scan_time = self.scan_stamp = None
        self.pub = self.create_publisher(Twist, p('output_topic'), 10)
        self.status = self.create_publisher(String, '~/status', 10)
        for key, topic in [('nav', p('nav_topic')), ('mission', p('mission_topic'))]:
            self.create_subscription(Twist, topic, lambda m, k=key: self.receive(k, m), 10)
        self.create_subscription(LaserScan, p('scan_topic'), self.scan_cb, qos_profile_sensor_data)
        self.create_service(SetBool, '~/mission_control', self.switch)
        # Gazebo pause中にも入力失効を処理する。
        self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, key, msg):
        vals = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
        self.commands[key] = (msg if all(math.isfinite(v) for v in vals) else Twist(), time.monotonic())

    def scan_cb(self, msg):
        if msg.ranges:
            self.scan_time = time.monotonic()
            self.scan_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

    def scan_ok(self):
        if self.scan_time is None:
            return False
        age = self.get_clock().now().nanoseconds*1e-9-self.scan_stamp
        return time.monotonic()-self.scan_time < self.scan_timeout and -0.1 <= age <= self.scan_timeout

    def switch(self, req, res):
        if req.data and not self.scan_ok():
            res.success, res.message = False, 'low_scanが未受信または古いです。'
            return res
        self.owner = req.data
        # 所有権変更前のバッファを再生しない。
        self.commands = {'nav': (Twist(), 0.), 'mission': (Twist(), 0.)}
        self.pub.publish(Twist())
        res.success, res.message = True, ('mission' if self.owner else 'nav')
        return res

    def tick(self):
        key = 'mission' if self.owner else 'nav'
        msg, stamp = self.commands[key]
        fresh = time.monotonic()-stamp < self.timeout
        ok = self.scan_ok()
        self.pub.publish(msg if fresh and ok else Twist())
        self.status.publish(String(data=json.dumps({'owner': key, 'scan_ok': ok, 'command_fresh': fresh})))


def main(args=None):
    rclpy.init(args=args)
    node = CommandGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()