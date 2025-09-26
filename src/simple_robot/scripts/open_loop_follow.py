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

        # 优化的控制参数
        self.goal_tolerance = 0.12
        self.angle_deadzone = math.radians(3)  # 稍微增大死区
        self.max_linear_speed = 0.6
        self.max_angular_speed = 1.0

        # 改进的PID控制参数
        self.kp_angular = 1.2
        self.ki_angular = 0.02
        self.kd_angular = 0.05

        # PID状态变量
        self.prev_angle_error = 0.0
        self.angle_error_integral = 0.0
        self.filtered_derivative = 0.0  # 新增：滤波后的微分项
        self.prev_time = time.time()

        # 平滑控制参数
        self.prev_angular_cmd = 0.0
        self.angular_smoothing = 0.7
        self.derivative_smoothing = 0.8  # 微分项平滑因子
        
        self.prev_linear_cmd = 0.0
        self.linear_smoothing = 0.8

        # 变化率限制
        self.max_angular_change = 0.8  # 最大角速度变化率 rad/s^2
        self.max_linear_change = 0.3   # 最大线速度变化率 m/s^2

        self.current_pose = None
        self.odom_received = False
        self.odom_count = 0  # 明确初始化
        self.last_log_time = 0
        
        # 控制定时器
        self.control_period = 0.15  # 控制周期
        self.timer = self.create_timer(self.control_period, self.follow_path)

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

        self.odom_count += 1
        if self.odom_count % 50 == 0:
            self.get_logger().info(
                f"Robot position: x={self.current_pose.position.x:.2f}, y={self.current_pose.position.y:.2f}, target: {self.current_path_index}/{len(self.path)}"
            )

    def pid_angular_control(self, angle_error, dt):
        """改进的PID角度控制器"""
        # 处理异常时间间隔
        if dt <= 0 or dt > 1.0:
            dt = self.control_period
            self.get_logger().warn(f"异常时间间隔: {dt:.3f}s, 使用默认值")

        # 死区处理 - 在死区内重置PID状态
        if abs(angle_error) < self.angle_deadzone:
            angle_error = 0.0
            self.angle_error_integral = 0.0  # 重置积分项
            self.filtered_derivative = 0.0   # 重置微分项
            return 0.0

        # 积分项（带抗饱和处理）
        self.angle_error_integral += angle_error * dt
        # 动态积分限制
        max_integral = 0.3 / self.ki_angular if self.ki_angular > 0.001 else 0.3
        self.angle_error_integral = max(-max_integral, min(max_integral, self.angle_error_integral))

        # 微分项带低通滤波
        raw_derivative = (angle_error - self.prev_angle_error) / dt if dt > 0.001 else 0.0
        self.filtered_derivative = (self.derivative_smoothing * self.filtered_derivative + 
                                   (1 - self.derivative_smoothing) * raw_derivative)

        # PID输出
        pid_output = (
            self.kp_angular * angle_error
            + self.ki_angular * self.angle_error_integral
            + self.kd_angular * self.filtered_derivative
        )

        # 更新历史值
        self.prev_angle_error = angle_error

        return pid_output

    def smooth_angular_command(self, new_angular_cmd):
        """带变化率限制的角速度平滑"""
        # 计算允许的最大变化
        max_change = self.max_angular_change * self.control_period
        
        # 限制变化率
        if abs(new_angular_cmd - self.prev_angular_cmd) > max_change:
            if new_angular_cmd > self.prev_angular_cmd:
                limited_cmd = self.prev_angular_cmd + max_change
            else:
                limited_cmd = self.prev_angular_cmd - max_change
        else:
            limited_cmd = new_angular_cmd
        
        # 应用指数平滑
        smoothed_cmd = (
            self.angular_smoothing * self.prev_angular_cmd
            + (1 - self.angular_smoothing) * limited_cmd
        )
        
        self.prev_angular_cmd = smoothed_cmd
        return smoothed_cmd

    def smooth_linear_command(self, new_linear_cmd):
        """带变化率限制的线速度平滑"""
        # 计算允许的最大变化
        max_change = self.max_linear_change * self.control_period
        
        # 限制变化率
        if abs(new_linear_cmd - self.prev_linear_cmd) > max_change:
            if new_linear_cmd > self.prev_linear_cmd:
                limited_cmd = self.prev_linear_cmd + max_change
            else:
                limited_cmd = self.prev_linear_cmd - max_change
        else:
            limited_cmd = new_linear_cmd
        
        # 应用指数平滑
        smoothed_cmd = (
            self.linear_smoothing * self.prev_linear_cmd
            + (1 - self.linear_smoothing) * limited_cmd
        )
        
        self.prev_linear_cmd = smoothed_cmd
        return smoothed_cmd

    def reset_pid_state(self):
        """重置PID状态"""
        self.angle_error_integral = 0.0
        self.prev_angle_error = 0.0
        self.filtered_derivative = 0.0
        self.prev_angular_cmd = 0.0
        self.prev_linear_cmd = 0.0

    def follow_path(self):
        if self.current_pose is None or not self.path:
            if self.current_pose is None:
                self.get_logger().warn("等待里程计数据...")
            return

        if self.current_path_index >= len(self.path):
            self.get_logger().info("🎯 路径跟踪完成!")
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
                f"✅ 到达点 {self.current_path_index + 1}/{len(self.path)}: ({target_x:.2f}, {target_y:.2f})"
            )
            self.current_path_index += 1
            
            # 重置PID状态
            self.reset_pid_state()
            
            # 发布停止指令
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.publisher_.publish(twist_msg)
            return

        # 时间差计算
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # 使用改进的PID控制角速度
        angular_vel = self.pid_angular_control(angle_error, dt)

        # 平滑角速度指令
        angular_vel = self.smooth_angular_command(angular_vel)

        # 根据角度误差和距离自适应调整线速度
        angle_factor = max(0.1, 1.0 - min(1.0, abs(angle_error) / (math.pi/2)))  # 保证最小速度
        distance_factor = min(1.0, distance_to_target / 0.8)  # 距离调节因子

        # 自适应线速度
        if abs(angle_error) > math.radians(45):  # 大角度转向时大幅减速
            base_linear_speed = 0.15
        elif abs(angle_error) > math.radians(20):  # 中等角度转向
            base_linear_speed = 0.3
        else:  # 小角度或直线
            base_linear_speed = self.max_linear_speed

        linear_vel = base_linear_speed * angle_factor * distance_factor
        linear_vel = max(0.08, min(self.max_linear_speed, linear_vel))  # 限制范围，保证最小速度

        # 平滑线速度指令
        linear_vel = self.smooth_linear_command(linear_vel)

        # 限制最终输出
        angular_vel = max(-self.max_angular_speed, min(self.max_angular_speed, angular_vel))

        # 调试信息
        if current_time - self.last_log_time > 2.0:
            self.get_logger().info(
                f"🎯 目标 {self.current_path_index + 1}/{len(self.path)}: ({target_x:.2f}, {target_y:.2f})\n"
                f"   距离: {distance_to_target:.2f}m, 角度误差: {math.degrees(angle_error):.1f}°\n"
                f"   控制指令: v={linear_vel:.2f}m/s, ω={angular_vel:.2f}rad/s"
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
        # 确保停止机器人
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        path_follower_node.publisher_.publish(twist_msg)
        time.sleep(0.5)  # 确保消息发布
        
        path_follower_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
