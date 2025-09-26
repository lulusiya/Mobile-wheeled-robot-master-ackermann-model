#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import os

# euler_from_quaternion 函数保持不变
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    """
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return 0.0, 0.0, yaw_z

class AckermannPathFollower(Node):
    def __init__(self, path_file):
        super().__init__("ackermann_path_follower")
        self.publisher_ = self.create_publisher(Twist, "/ackermann_steering_controller/reference_unstamped", 10)
        
        # 订阅里程计，我们需要持续的反馈
        self.odom_subscriber = self.create_subscription(
            Odometry, 
            "/ackermann_steering_controller/odometry", 
            self.odom_callback, 
            10
        )
        
        self.path = self.load_path(path_file)
        self.current_path_index = 0
        self.current_pose = None
        self.odom_received = False

        # --- 控制参数 ---
        self.MAX_LINEAR_SPEED = 0.5  # 机器人最大前进速度 (m/s)
        self.MAX_ANGULAR_SPEED = 0.8 # 机器人最大转向率 (rad/s)
        self.GOAL_TOLERANCE = 0.15   # 到达目标的距离容差 (m)
        
        # 比例控制器的增益 (这是唯一需要调整的关键参数!)
        self.KP_ANGULAR = 1.2

        # 使用高频定时器进行闭环控制
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Ackermann path follower initialized.")

    def load_path(self, file_path):
        # (这个函数和您原来的一样，保持不变)
        path = []
        try:
            with open(file_path, "r") as f:
                for line in f:
                    parts = line.strip().split(",")
                    if len(parts) == 2:
                        path.append((float(parts[0]), float(parts[1])))
            self.get_logger().info(f"Loaded {len(path)} points from {file_path}")
        except FileNotFoundError:
            self.get_logger().error(f"Path file not found: {file_path}")
        return path

    def odom_callback(self, msg):
        # 持续更新机器人的当前位姿
        self.current_pose = msg.pose.pose
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info("Odometry received. Starting control loop.")

    def control_loop(self):
        # 如果没有收到里程计或路径已走完，则不执行任何操作
        if not self.odom_received or self.current_path_index >= len(self.path):
            return

        # 获取当前位置和朝向
        current_position = self.current_pose.position
        _, _, current_yaw = euler_from_quaternion(
            self.current_pose.orientation.x, self.current_pose.orientation.y,
            self.current_pose.orientation.z, self.current_pose.orientation.w
        )

        # 获取当前目标点
        target_x, target_y = self.path[self.current_path_index]

        # 计算到目标的距离
        distance_to_target = math.sqrt((target_x - current_position.x)**2 + (target_y - current_position.y)**2)

        # 检查是否已到达目标点
        if distance_to_target < self.GOAL_TOLERANCE:
            self.get_logger().info(f"Reached waypoint {self.current_path_index + 1}/{len(self.path)}")
            self.current_path_index += 1
            if self.current_path_index >= len(self.path):
                self.get_logger().info("Path completed!")
                self.stop_robot()
                self.timer.cancel()
            return

        # --- 核心控制逻辑 ---
        # 1. 计算目标角度和角度误差
        angle_to_target = math.atan2(target_y - current_position.y, target_x - current_position.x)
        angle_error = angle_to_target - current_yaw
        # 归一化误差
        while angle_error > math.pi: angle_error -= 2 * math.pi
        while angle_error < -math.pi: angle_error += 2 * math.pi
        
        # 2. 根据角度误差计算转向指令 (比例控制)
        #    这是您想法的直接体现：根据所需角度，直接计算转向指令
        angular_vel = self.KP_ANGULAR * angle_error
        
        # 3. 限制转向指令，防止过快转向
        angular_vel = max(-self.MAX_ANGULAR_SPEED, min(self.MAX_ANGULAR_SPEED, angular_vel))
        
        # 4. 自适应调整前进速度：转弯越大，速度越慢
        linear_vel = self.MAX_LINEAR_SPEED * (1.0 - 0.8 * abs(angle_error) / math.pi)
        
        # 创建并发布 Twist 消息
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.publisher_.publish(twist_msg)

        self.get_logger().info(
            f"Target {self.current_path_index+1}: dist={distance_to_target:.2f}m, angle_err={math.degrees(angle_error):.1f}°, "
            f"cmd_vel: v={linear_vel:.2f}, w={angular_vel:.2f}",
            throttle_duration_sec=1.0 # 每秒只打印一次日志，防止刷屏
        )

    def stop_robot(self):
        self.publisher_.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    # main 函数和原来的一样
    workspace_root = os.getcwd()
    path_file_path = os.path.join(workspace_root, "path.txt")
    if not os.path.exists(path_file_path):
        print("path.txt not found!")
        return
    path_follower_node = AckermannPathFollower(path_file=path_file_path)
    rclpy.spin(path_follower_node)
    path_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
