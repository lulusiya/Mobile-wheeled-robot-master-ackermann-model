#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive  # 不带时间戳的版本
from nav_msgs.msg import Odometry  # 里程计消息
from geometry_msgs.msg import Twist  # 导入正确的消息类型！
import time
import sys  # 导入 sys 模块
import math
import json
import os


class TaskExecutor(Node):
    """
    智能任务执行节点
    任务：导航到目标位置，记录完整路径，检测到达终点后退出
    """

    def __init__(self):
        super().__init__("task_executor")

        # 声明和获取参数
        self.declare_parameter("goal_x", 5.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_z", 0.1)

        self.goal_x = self.get_parameter("goal_x").get_parameter_value().double_value
        self.goal_y = self.get_parameter("goal_y").get_parameter_value().double_value
        self.goal_z = self.get_parameter("goal_z").get_parameter_value().double_value

        # 创建发布器 - 现在发布Twist消息
        self.cmd_publisher = self.create_publisher(
            Twist,  # <--- 修改消息类型
            "/ackermann_steering_controller/reference_unstamped",
            10,
        )

        # 创建多个订阅器，监听所有可能的里程计话题
        self.odom_subscribers = []
        self.active_odom_topic = None
        self.topic_to_subscriber = {}  # 话题名到订阅器的映射
        possible_odom_topics = [
            "/odom",
            "/ackermann_steering_controller/odometry",
            "/odometry/filtered",
        ]

        for topic in possible_odom_topics:
            # 创建专门的回调函数而不是使用lambda
            def create_callback(topic_name):
                def callback(msg):
                    self.odom_callback(msg, topic_name)

                return callback

            sub = self.create_subscription(Odometry, topic, create_callback(topic), 10)
            self.odom_subscribers.append(sub)
            self.topic_to_subscriber[topic] = sub
            self.get_logger().info(f"创建里程计订阅器: {topic}")

        # 机器人状态
        self.current_position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.current_orientation = {"yaw": 0.0}
        self.start_time = None
        self.task_started = False
        self.goal_reached = False
        self.wait_count = 0
        self.odom_received = False  # 标记是否接收到里程计数据

        # 路径记录
        self.path_data = []
        self.last_record_time = time.time()
        self.record_interval = 1.0  # 每秒记录一次位置

        # 导航参数
        self.goal_tolerance = 0.5  # 到达目标的距离阈值（米）
        self.max_speed = 1.5  # 最大速度 m/s
        self.min_speed = 0.3  # 最小速度 m/s
        self.max_duration = 60.0  # 最大任务时间60秒

        # 定时器
        self.check_timer = self.create_timer(1.0, self.check_system_ready)
        self.control_timer = None
        self.path_record_timer = None

        self.get_logger().info(
            f"任务执行器已启动！目标位置: ({self.goal_x:.2f}, {self.goal_y:.2f})"
        )
        self.get_logger().info(
            "发布控制指令到: /ackermann_steering_controller/reference_unstamped"
        )

    def check_system_ready(self):
        """等待系统启动完成"""
        self.wait_count += 1

        if self.wait_count >= 10:  # 等待10秒让所有组件启动
            if not self.task_started:
                if not self.odom_received:
                    self.get_logger().warn("⚠️ 还未接收到里程计数据，继续等待...")
                    if self.wait_count >= 25:  # 最多等待25秒
                        self.get_logger().error("❌ 超时未接收到里程计数据，请检查话题")
                        self.finish_task(success=False, reason="no_odometry")
                    return

                self.task_started = True
                self.start_time = time.time()

                # 销毁检查定时器，创建控制和路径记录定时器
                self.check_timer.destroy()
                self.control_timer = self.create_timer(
                    0.1, self.control_callback
                )  # 10Hz控制频率
                self.path_record_timer = self.create_timer(
                    1.0, self.record_path_point
                )  # 1Hz记录频率

                self.get_logger().info(
                    f"系统准备完成，开始导航到目标位置 ({self.goal_x:.2f}, {self.goal_y:.2f})..."
                )
        else:
            status_msg = f"等待系统启动... ({self.wait_count}/10 秒)"
            if self.odom_received:
                status_msg += " [里程计: ✅]"
            else:
                status_msg += " [里程计: ❌]"
            self.get_logger().info(status_msg)

    def odom_callback(self, msg, topic_name):
        """处理里程计数据"""
        # 首次接收里程计数据时记录，并销毁其他不用的订阅器
        if not self.odom_received:
            self.odom_received = True
            self.active_odom_topic = topic_name
            self.get_logger().info(
                f"✅ 首次接收到里程计数据！来自话题: {self.active_odom_topic}"
            )

            # 销毁其他订阅器
            for topic, sub in self.topic_to_subscriber.items():
                if topic != self.active_odom_topic:
                    self.destroy_subscription(sub)
                    self.get_logger().info(f"销毁不活跃的订阅器: {topic}")

            # 更新订阅器列表，只保留活跃的
            self.odom_subscribers = [self.topic_to_subscriber[self.active_odom_topic]]
            self.topic_to_subscriber = {
                self.active_odom_topic: self.topic_to_subscriber[self.active_odom_topic]
            }

        # 只处理来自活跃话题的数据
        if topic_name != self.active_odom_topic:
            return

        # 提取位置信息
        self.current_position["x"] = msg.pose.pose.position.x
        self.current_position["y"] = msg.pose.pose.position.y
        self.current_position["z"] = msg.pose.pose.position.z

        # 提取朝向信息 (四元数转欧拉角)
        orientation_q = msg.pose.pose.orientation
        self.current_orientation["yaw"] = self.quaternion_to_yaw(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )

    def quaternion_to_yaw(self, x, y, z, w):
        """将四元数转换为偏航角 (yaw)"""
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def record_path_point(self):
        """记录当前位置到路径数据中"""
        if self.task_started and not self.goal_reached:
            current_time = time.time()
            elapsed_time = current_time - self.start_time

            path_point = {
                "timestamp": current_time,
                "elapsed_time": round(elapsed_time, 2),
                "x": round(self.current_position["x"], 3),
                "y": round(self.current_position["y"], 3),
                "z": round(self.current_position["z"], 3),
                "yaw": round(self.current_orientation["yaw"], 3),
            }

            self.path_data.append(path_point)

            # 每10个点打印一次进度
            if len(self.path_data) % 10 == 0:
                distance_to_goal = self.calculate_distance_to_goal()
                self.get_logger().info(
                    f"路径点 {len(self.path_data)}: 位置 ({path_point['x']:.2f}, {path_point['y']:.2f}), "
                    f"距离目标 {distance_to_goal:.2f}m"
                )

    def calculate_distance_to_goal(self):
        """计算到目标位置的距离"""
        dx = self.goal_x - self.current_position["x"]
        dy = self.goal_y - self.current_position["y"]
        return math.sqrt(dx * dx + dy * dy)

    def calculate_angle_to_goal(self):
        """计算到目标位置的角度"""
        dx = self.goal_x - self.current_position["x"]
        dy = self.goal_y - self.current_position["y"]
        return math.atan2(dy, dx)

    def control_callback(self):
        """主控制循环 - 导航到目标位置"""
        if not self.task_started or self.goal_reached:
            return

        elapsed_time = time.time() - self.start_time

        # 检查超时
        if elapsed_time > self.max_duration:
            self.get_logger().warn(f"任务超时 ({self.max_duration}秒)，停止任务")
            self.finish_task(success=False, reason="timeout")
            return

        # 计算到目标的距离和角度
        distance_to_goal = self.calculate_distance_to_goal()
        angle_to_goal = self.calculate_angle_to_goal()
        current_yaw = self.current_orientation["yaw"]

        # 检查是否到达目标
        if distance_to_goal <= self.goal_tolerance:
            self.get_logger().info(f"成功到达目标位置！距离: {distance_to_goal:.3f}m")
            self.finish_task(success=True, reason="goal_reached")
            return

        # 计算转向角度
        angle_diff = angle_to_goal - current_yaw
        # 将角度差限制在 [-π, π] 范围内
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # 计算控制指令 - Twist消息
        # 转向是通过角速度z来控制的，线速度x控制前进
        angle_diff_normalized = max(
            -1.0, min(1.0, angle_diff / (math.pi / 2))
        )  # 将角度差归一化到-1到1

        # 根据距离调整速度
        if distance_to_goal > 3.0:
            speed = self.max_speed
        elif distance_to_goal > 1.0:
            speed = self.max_speed * 0.7
        else:
            speed = self.min_speed

        # 如果需要大幅转向，降低速度
        if abs(angle_diff_normalized) > 0.5:
            speed *= 0.5

        # 发布Twist控制指令
        cmd_msg = Twist()
        cmd_msg.linear.x = speed
        cmd_msg.angular.z = angle_diff_normalized * 1.5  # 乘以一个系数来调整转向灵敏度

        self.cmd_publisher.publish(cmd_msg)

        # 每5秒打印一次控制信息
        if int(elapsed_time) % 5 == 0 and int(elapsed_time) != int(elapsed_time - 0.1):
            self.get_logger().info(
                f"🚗 控制指令: 速度={cmd_msg.linear.x:.2f}m/s, 转向={cmd_msg.angular.z:.2f}rad/s, "
                f"当前位置=({self.current_position['x']:.2f}, {self.current_position['y']:.2f})"
            )

    def save_path_data(self):
        """保存路径数据到文件"""
        try:
            # 创建路径数据目录
            path_dir = "/tmp/robot_paths"
            os.makedirs(path_dir, exist_ok=True)

            # 生成文件名
            timestamp = int(time.time())
            filename = f"path_{timestamp}.json"
            filepath = os.path.join(path_dir, filename)

            # 准备完整的路径数据
            path_info = {
                "start_position": (
                    {"x": self.path_data[0]["x"], "y": self.path_data[0]["y"]}
                    if self.path_data
                    else {"x": 0, "y": 0}
                ),
                "goal_position": {"x": self.goal_x, "y": self.goal_y},
                "total_points": len(self.path_data),
                "total_duration": (
                    self.path_data[-1]["elapsed_time"] if self.path_data else 0
                ),
                "path_points": self.path_data,
            }

            # 写入文件
            with open(filepath, "w") as f:
                json.dump(path_info, f, indent=2)

            self.get_logger().info(f"路径数据已保存到: {filepath}")
            return filepath

        except Exception as e:
            self.get_logger().error(f"保存路径数据失败: {e}")
            return None

    def finish_task(self, success=True, reason="completed"):
        """完成任务并退出"""
        if self.goal_reached:
            return  # 避免重复调用

        self.goal_reached = True

        # 停止机器人
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_publisher.publish(stop_msg)

        # 记录最后一个路径点
        self.record_path_point()

        # 保存路径数据
        path_file = self.save_path_data()

        # 清理定时器
        if self.control_timer:
            self.control_timer.destroy()
        if self.path_record_timer:
            self.path_record_timer.destroy()

        # 打印任务总结
        total_time = time.time() - self.start_time if self.start_time else 0
        final_distance = self.calculate_distance_to_goal()

        self.get_logger().info("=" * 50)
        if success:
            self.get_logger().info("✅ 任务成功完成！")
        else:
            self.get_logger().warn(f"⚠️ 任务未完成: {reason}")

        self.get_logger().info("📊 任务统计:")
        self.get_logger().info(f"   总用时: {total_time:.2f} 秒")
        self.get_logger().info(f"   路径点数: {len(self.path_data)}")
        self.get_logger().info(f"   最终距目标: {final_distance:.3f} m")
        self.get_logger().info(f"   目标位置: ({self.goal_x:.2f}, {self.goal_y:.2f})")
        if path_file:
            self.get_logger().info(f"   路径文件: {path_file}")
        self.get_logger().info("=" * 50)

        # 延迟1秒后退出，确保所有消息都发送了
        time.sleep(1)
        self.destroy_node()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    task_executor = TaskExecutor()
    rclpy.spin(task_executor)


if __name__ == "__main__":
    main()
