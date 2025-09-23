#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import os


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


class PathFollower(Node):
    def __init__(self, path_file):
        super().__init__("path_follower")
        self.publisher_ = self.create_publisher(
            Twist, "/ackermann_steering_controller/reference_unstamped", 10
        )

        # Subscribe to multiple possible odometry topics
        self.odom_subscribers = []
        self.active_odom_topic = None
        possible_odom_topics = [
            "/odom",
            "/ackermann_steering_controller/odometry",
            "/odometry/filtered",
        ]

        for topic in possible_odom_topics:

            def create_callback(topic_name):
                def callback(msg):
                    self.odom_callback(msg, topic_name)

                return callback

            sub = self.create_subscription(Odometry, topic, create_callback(topic), 10)
            self.odom_subscribers.append(sub)
            self.get_logger().info(f"Subscribed to odometry topic: {topic}")

        self.path = self.load_path(path_file)
        self.current_path_index = 0

        # 优化的控制参数 - 减少抖动
        self.goal_tolerance = 0.12  # 稍微放宽容差
        self.angle_deadzone = math.radians(5)  # 增大死区到5度，减少微小抖动
        self.max_linear_speed = 0.6  # 降低最大速度，提高稳定性
        self.max_angular_speed = 0.8  # 降低最大角速度，减少转向过度

        # 更温和的PID控制参数
        self.kp_angular = 0.8  # 降低比例增益，减少响应过度
        self.ki_angular = 0.02  # 进一步降低积分增益
        self.kd_angular = 0.05  # 降低微分增益，减少高频振荡

        # PID状态变量
        self.prev_angle_error = 0.0
        self.angle_error_integral = 0.0
        self.prev_time = time.time()

        # 增强平滑控制
        self.prev_angular_cmd = 0.0
        self.angular_smoothing = 0.85  # 增加平滑因子，更强的平滑效果

        # 速度平滑
        self.prev_linear_cmd = 0.0
        self.linear_smoothing = 0.8  # 添加线速度平滑

        self.current_pose = None
        self.odom_received = False
        self.last_log_time = 0
        # 降低控制频率，减少指令更新频率
        self.timer = self.create_timer(0.15, self.follow_path)  # 从10Hz降到6.7Hz

        self.get_logger().info(
            f"Path follower initialized with {len(self.path)} waypoints"
        )
        if self.path:
            self.get_logger().info(f"First waypoint: {self.path[0]}")
            self.get_logger().info(f"Last waypoint: {self.path[-1]}")
        self.get_logger().info(
            "Publishing to: /ackermann_steering_controller/reference_unstamped"
        )

    def load_path(self, file_path):
        path = []
        try:
            with open(file_path, "r") as f:
                for line in f:
                    parts = line.strip().split(",")
                    if len(parts) == 2:
                        try:
                            x = float(parts[0])
                            y = float(parts[1])
                            path.append((x, y))
                        except ValueError:
                            self.get_logger().warn(
                                f"Cannot parse line: {line.strip()}, skipping."
                            )
            self.get_logger().info(f"Loaded {len(path)} points from {file_path}")
        except FileNotFoundError:
            self.get_logger().error(f"Path file not found: {file_path}")
        return path

    def odom_callback(self, msg, topic_name):
        self.current_pose = msg.pose.pose
        if not self.odom_received:
            self.get_logger().info(f"First odometry data received from: {topic_name}")
            self.active_odom_topic = topic_name
            self.odom_received = True

        if not hasattr(self, "odom_count"):
            self.odom_count = 0
        self.odom_count += 1
        if self.odom_count % 50 == 0:
            self.get_logger().info(
                f"Robot position: x={self.current_pose.position.x:.2f}, y={self.current_pose.position.y:.2f}, target: {self.current_path_index}/{len(self.path)}"
            )

    def pid_angular_control(self, angle_error, dt):
        """PID角度控制器"""
        # 死区处理
        if abs(angle_error) < self.angle_deadzone:
            angle_error = 0.0

        # 积分项（防止积分饱和）
        self.angle_error_integral += angle_error * dt
        self.angle_error_integral = max(-0.5, min(0.5, self.angle_error_integral))

        # 微分项
        angle_error_derivative = (
            (angle_error - self.prev_angle_error) / dt if dt > 0 else 0.0
        )

        # PID输出
        pid_output = (
            self.kp_angular * angle_error
            + self.ki_angular * self.angle_error_integral
            + self.kd_angular * angle_error_derivative
        )

        # 更新历史值
        self.prev_angle_error = angle_error

        return pid_output

    def smooth_angular_command(self, new_angular_cmd):
        """平滑角速度指令"""
        smoothed_cmd = (
            self.angular_smoothing * self.prev_angular_cmd
            + (1 - self.angular_smoothing) * new_angular_cmd
        )
        self.prev_angular_cmd = smoothed_cmd
        return smoothed_cmd

    def smooth_linear_command(self, new_linear_cmd):
        """平滑线速度指令"""
        smoothed_cmd = (
            self.linear_smoothing * self.prev_linear_cmd
            + (1 - self.linear_smoothing) * new_linear_cmd
        )
        self.prev_linear_cmd = smoothed_cmd
        return smoothed_cmd

    def follow_path(self):
        if self.current_pose is None or not self.path:
            return

        if self.current_path_index >= len(self.path):
            self.get_logger().info("🎯 Path completed!")
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.publisher_.publish(twist_msg)
            self.timer.cancel()
            return

        # 获取当前位置和朝向
        current_position = self.current_pose.position
        orientation_q = self.current_pose.orientation
        _, _, current_yaw = euler_from_quaternion(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )

        target_x, target_y = self.path[self.current_path_index]

        # 计算到目标的距离和角度
        dx = target_x - current_position.x
        dy = target_y - current_position.y
        distance_to_target = math.sqrt(dx**2 + dy**2)
        angle_to_target = math.atan2(dy, dx)

        # 角度误差归一化
        angle_error = angle_to_target - current_yaw
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # 检查是否到达当前目标点
        if distance_to_target < self.goal_tolerance:
            self.get_logger().info(
                f"✅ Reached point {self.current_path_index + 1}/{len(self.path)}: ({target_x:.2f}, {target_y:.2f})"
            )
            self.current_path_index += 1
            # 重置PID状态，避免积分项累积
            self.angle_error_integral = 0.0
            self.prev_angle_error = 0.0
            return

        # 时间差计算
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # 使用PID控制角速度
        angular_vel = self.pid_angular_control(angle_error, dt)

        # 平滑角速度指令
        angular_vel = self.smooth_angular_command(angular_vel)

        # 根据角度误差和距离自适应调整线速度
        angle_factor = 1.0 - min(1.0, abs(angle_error) / math.pi)  # 角度越大，速度越小
        distance_factor = min(1.0, distance_to_target / 0.5)  # 距离调节因子

        # 基础线速度
        if abs(angle_error) > math.radians(30):  # 大角度转向时减速
            base_linear_speed = 0.2
        elif abs(angle_error) > math.radians(10):  # 中等角度转向
            base_linear_speed = 0.4
        else:  # 小角度或直线
            base_linear_speed = self.max_linear_speed

        linear_vel = base_linear_speed * angle_factor * distance_factor
        linear_vel = max(0.1, min(self.max_linear_speed, linear_vel))  # 限制范围

        # 平滑线速度指令
        linear_vel = self.smooth_linear_command(linear_vel)

        # 限制角速度
        angular_vel = max(
            -self.max_angular_speed, min(self.max_angular_speed, angular_vel)
        )

        # 调试信息
        if current_time - self.last_log_time > 1.5:
            self.get_logger().info(
                f"🎯 Target {self.current_path_index + 1}/{len(self.path)}: ({target_x:.2f}, {target_y:.2f}), "
                f"dist: {distance_to_target:.2f}m, angle_err: {math.degrees(angle_error):.1f}°, "
                f"cmd: v={linear_vel:.2f}, ω={angular_vel:.2f}"
            )
            self.last_log_time = current_time

        # 发布控制指令
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.publisher_.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)

    workspace_root = os.getcwd()
    path_file_path = os.path.join(workspace_root, "path.txt")

    print(f"Looking for path file at: {path_file_path}")
    if not os.path.exists(path_file_path):
        print(f"Error: path.txt not found at {path_file_path}")
        print("Please make sure path.txt exists in the workspace root directory")
        return

    path_follower_node = PathFollower(path_file=path_file_path)

    try:
        rclpy.spin(path_follower_node)
    except KeyboardInterrupt:
        print("Path follower stopped by user")
    finally:
        path_follower_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
