import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


@pytest.mark.launch_test
def generate_test_description():
    decision = launch_ros.actions.Node(
        package="team01_race", executable="decision_node", name="team01_decision",
        parameters=[{"use_sim_time": False, "scan_topic": "/test/filtered_scan",
                     "race_start_topic": "/test/start", "requested_speed_topic": "/test/requested_speed",
                     "requested_steering_topic": "/test/requested_steering",
                     "controller_ready_topic": "/test/controller_ready",
                     "safety.sensor_timeout_sec": 0.5}],
    )
    control = launch_ros.actions.Node(
        package="team01_race", executable="control_adapter_node", name="team01_control_adapter",
        parameters=[{"use_sim_time": False, "requested_speed_topic": "/test/requested_speed",
                     "requested_steering_topic": "/test/requested_steering",
                     "controller_ready_topic": "/test/controller_ready",
                     "race_start_topic": "/test/start", "speed_topic": "/test/speed",
                     "steering_topic": "/test/steering", "scan_topic": "/test/raw_scan",
                     "safety.sensor_timeout_sec": 0.5, "safety.command_timeout_sec": 0.5,
                     "control.max_accel_mps2": 100.0, "control.max_decel_mps2": 100.0,
                     "control.max_steering_rate_radps": 100.0}],
    )
    return launch.LaunchDescription([decision, control, launch_testing.actions.ReadyToTest()])


class TestRaceStartGate(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("team01_start_gate_test")
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        start_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.VOLATILE)
        self.filtered_pub = self.node.create_publisher(LaserScan, "/test/filtered_scan", sensor_qos)
        self.raw_pub = self.node.create_publisher(LaserScan, "/test/raw_scan", sensor_qos)
        self.start_pub = self.node.create_publisher(Bool, "/test/start", start_qos)
        self.speed = []
        self.steering = []
        self.node.create_subscription(Float32, "/test/speed", lambda m: self.speed.append(m.data), 10)
        self.node.create_subscription(Float32, "/test/steering", lambda m: self.steering.append(m.data), 10)

    def tearDown(self):
        self.node.destroy_node()

    @staticmethod
    def scan():
        msg = LaserScan()
        msg.angle_min = -math.pi / 2
        msg.angle_max = math.pi / 2
        msg.angle_increment = math.pi / 180
        msg.range_min = 0.04
        msg.range_max = 6.0
        msg.ranges = [float("inf")] * 181
        for degree in range(-90, 91):
            if abs(degree) < 9 or degree == 0:
                continue
            angle = math.radians(degree)
            distance = 0.75 / abs(math.sin(angle))
            if distance * math.cos(angle) <= 5.0 and distance <= msg.range_max:
                msg.ranges[degree + 90] = distance
        return msg

    def spin_for(self, seconds, publish_scan=True):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if publish_scan:
                msg = self.scan()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                self.filtered_pub.publish(msg)
                self.raw_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            time.sleep(0.02)

    def test_false_true_false_controls_final_topics(self):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and self.start_pub.get_subscription_count() < 2:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual(self.start_pub.get_subscription_count(), 2)

        self.spin_for(0.4)
        self.assertTrue(self.speed)
        self.assertEqual(self.speed[-1], 0.0)

        self.start_pub.publish(Bool(data=False))
        self.spin_for(0.2)
        self.assertEqual(self.speed[-1], 0.0)

        self.start_pub.publish(Bool(data=True))
        self.spin_for(0.15, publish_scan=False)
        self.assertEqual(self.speed[-1], 0.0)

        self.spin_for(0.5, publish_scan=True)
        self.assertGreater(self.speed[-1], 0.0)
        self.assertAlmostEqual(self.speed[-1], 5.0, delta=0.05)
        self.assertTrue(all(abs(value) <= 0.523599 + 1e-6 for value in self.steering))

        self.start_pub.publish(Bool(data=False))
        self.spin_for(0.15, publish_scan=False)
        self.assertEqual(self.speed[-1], 0.0)
        self.assertEqual(self.steering[-1], 0.0)
