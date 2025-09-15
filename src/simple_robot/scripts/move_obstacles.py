#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class ObstacleController(Node):
    def __init__(self):
        super().__init__("obstacle_controller")

        # 创建发布器来控制障碍物运动
        self.box_publisher = self.create_publisher(Twist, "/box_cmd_vel", 10)
        self.cylinder_publisher = self.create_publisher(Twist, "/cylinder_cmd_vel", 10)

        # 创建定时器，每0.1秒执行一次
        self.timer = self.create_timer(0.1, self.control_obstacles)

        # 初始化运动参数
        self.start_time = time.time()

        self.get_logger().info("障碍物控制器已启动")

    def control_obstacles(self):
        current_time = time.time() - self.start_time

        # 创建速度消息
        box_twist = Twist()
        cylinder_twist = Twist()

        # 红色箱子：左右往返运动 (sin波形)
        # 速度范围：-1.0 到 1.0 m/s
        box_twist.linear.x = 1.0 * math.sin(current_time * 0.5)  # 0.5 rad/s 频率
        box_twist.linear.y = 0.0
        box_twist.angular.z = 0.0

        # 蓝色圆柱体：前后往返运动 (cos波形，相位差)
        cylinder_twist.linear.x = 0.0
        cylinder_twist.linear.y = 1.0 * math.cos(
            current_time * 0.3
        )  # 0.3 rad/s 频率，不同频率
        cylinder_twist.angular.z = 0.0

        # 发布运动命令
        self.box_publisher.publish(box_twist)
        self.cylinder_publisher.publish(cylinder_twist)


def main(args=None):
    rclpy.init(args=args)

    obstacle_controller = ObstacleController()

    try:
        rclpy.spin(obstacle_controller)
    except KeyboardInterrupt:
        pass

    obstacle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

