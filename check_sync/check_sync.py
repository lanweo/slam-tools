#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
# 导入imu消息类型
from sensor_msgs.msg import Imu  # 替换为实际消息类型（如Imu）
# 导入image消息类型
from sensor_msgs.msg import Image  # 替换为实际消息类型（如Image）
class SyncChecker(Node):
    def __init__(self):
        super().__init__('sync_checker')
        self.sub = self.create_subscription(
            Imu,  # 替换为实际消息类型（如LaserScan）
            '/sensing/imu/tamagawa/imu_raw1',
            self.callback,
            10)
        self.delays = []

    def callback(self, msg):
        sensor_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        current_time = self.get_clock().now().nanoseconds * 1e-9
        delay = current_time - sensor_time
        self.delays.append(delay)
        self.get_logger().info(f"Delay: {delay:.6f} seconds")
        # 打印系统时间戳与传感器时间戳
        self.get_logger().info(f"Sensor time: {sensor_time:.6f}, Current time: {current_time:.6f}")

    def print_stats(self):
        if self.delays:
            avg = sum(self.delays) / len(self.delays)
            self.get_logger().info(f"Average delay: {avg:.6f} sec")

if __name__ == '__main__':
    rclpy.init()
    node = SyncChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_stats()
        node.destroy_node()