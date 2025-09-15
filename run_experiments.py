#!/usr/bin/env python3
"""
自动化批量仿真实验脚本
用于连续执行指定次数的机器人仿真，每次使用不同的地图和起始位置
"""

import subprocess
import os
import random
import time
import csv
import json
import datetime

# =================================================================================
# ============================ 1. 主要参数修改位置 ============================
# =================================================================================

# 总共要运行的仿真次数 - 您可以修改这个数字
NUM_RUNS = 10000

# 您的 ROS 2 包名和启动文件名 - 如果您的包名不同，请修改这里
ROS_PACKAGE_NAME = "simple_robot"
ROS_LAUNCH_FILE = "robot.launch.py"

# 存放 .world 文件的目录路径 (相对于本脚本所在的项目根目录)
WORLDS_DIRECTORY = "src/simple_robot/worlds"

# =================================================================================
# ======== 2. 坐标范围修改位置 (根据您地图的实际大小和复杂性进行调整) ========
# =================================================================================

# X 坐标的随机范围 [min, max] - 根据您的地图大小调整
X_RANGE = [-8.0, 8.0]

# Y 坐标的随机范围 [min, max] - 根据您的地图大小调整
Y_RANGE = [-8.0, 8.0]

# 偏航角的随机范围 [min, max] (弧度制) - 0 到 2π
YAW_RANGE = [0.0, 6.28]

# 两次仿真之间的等待时间(秒) - 给系统喘息时间
SLEEP_BETWEEN_RUNS = 2

# =================================================================================
# ================= 3. 数据记录配置 (您可以根据需要调整) ===================
# =================================================================================

# 是否启用数据记录
ENABLE_LOGGING = True

# 数据记录输出目录
LOG_DIRECTORY = "experiment_logs"

# CSV 汇总文件名 (包含每次运行的基本统计)
CSV_LOG_FILE = "experiment_summary.csv"

# JSON 详细文件名 (包含每次运行的完整数据)
JSON_LOG_FILE = "experiment_details.json"


# =================================================================================
# ============================= 脚本主逻辑 (一般无需修改) =========================
# =================================================================================


class ExperimentLogger:
    """
    实验数据记录器 - 负责记录每次仿真的详细数据
    """

    def __init__(self):
        self.experiment_data = []
        self.csv_file_path = None
        self.json_file_path = None

        if ENABLE_LOGGING:
            self._setup_logging()

    def _setup_logging(self):
        """设置日志记录环境"""
        # 创建日志目录
        if not os.path.exists(LOG_DIRECTORY):
            os.makedirs(LOG_DIRECTORY)
            print(f"📁 创建日志目录: {LOG_DIRECTORY}")

        # 生成带时间戳的文件名
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file_path = os.path.join(LOG_DIRECTORY, f"{timestamp}_{CSV_LOG_FILE}")
        self.json_file_path = os.path.join(
            LOG_DIRECTORY, f"{timestamp}_{JSON_LOG_FILE}"
        )

        # 创建CSV文件并写入表头
        with open(self.csv_file_path, "w", newline="", encoding="utf-8") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(
                [
                    "Run_Number",
                    "Timestamp",
                    "World_File",
                    "Start_X",
                    "Start_Y",
                    "Start_Z",
                    "Start_Yaw",
                    "Goal_X",  # 新增：终点X坐标
                    "Goal_Y",  # 新增：终点Y坐标
                    "Goal_Z",  # 新增：终点Z坐标
                    "Status",
                    "Duration_Seconds",
                    "Exit_Code",
                    "Error_Message",
                    "Path_Points_Count",  # 新增：路径点数量
                ]
            )

        print(f"📊 CSV 日志文件: {self.csv_file_path}")
        print(f"📄 JSON 日志文件: {self.json_file_path}")

    def log_simulation_run(
        self,
        run_number,
        world_file,
        start_pos,
        goal_pos,  # 新增：终点位置
        status,
        duration,
        exit_code=None,
        error_message=None,
        path_data=None,  # 新增：路径数据
    ):
        """记录单次仿真运行数据"""
        if not ENABLE_LOGGING:
            return

        timestamp = datetime.datetime.now().isoformat()

        # 处理路径数据
        if path_data is None:
            path_data = []

        # 准备数据
        run_data = {
            "run_number": run_number,
            "timestamp": timestamp,
            "world_file": os.path.basename(world_file),
            "world_file_full_path": world_file,
            "start_position": start_pos,
            "goal_position": goal_pos,  # 新增：终点位置
            "status": status,  # 'success', 'failed', 'timeout', 'error'
            "duration_seconds": round(duration, 2),
            "exit_code": exit_code,
            "error_message": error_message,
            "path_data": path_data,  # 新增：完整路径数据
            "path_points_count": len(path_data),  # 新增：路径点数量
        }

        # 添加到内存中的数据列表
        self.experiment_data.append(run_data)

        # 写入CSV文件 (立即写入，防止程序意外终止丢失数据)
        with open(self.csv_file_path, "a", newline="", encoding="utf-8") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(
                [
                    run_number,
                    timestamp,
                    os.path.basename(world_file),
                    start_pos["x"],
                    start_pos["y"],
                    start_pos["z"],
                    start_pos["yaw"],
                    goal_pos["x"],  # 新增：终点坐标
                    goal_pos["y"],
                    goal_pos["z"],
                    status,
                    round(duration, 2),
                    exit_code,
                    error_message or "",
                    len(path_data),  # 新增：路径点数量
                ]
            )

        # 写入JSON文件 (覆盖式写入，保持最新的完整数据)
        with open(self.json_file_path, "w", encoding="utf-8") as jsonfile:
            json.dump(self.experiment_data, jsonfile, indent=2, ensure_ascii=False)

    def get_summary_stats(self):
        """获取汇总统计信息"""
        if not self.experiment_data:
            return {}

        total_runs = len(self.experiment_data)
        successful_runs = len(
            [r for r in self.experiment_data if r["status"] == "success"]
        )
        failed_runs = len([r for r in self.experiment_data if r["status"] == "failed"])
        timeout_runs = len(
            [r for r in self.experiment_data if r["status"] == "timeout"]
        )
        error_runs = len([r for r in self.experiment_data if r["status"] == "error"])

        durations = [
            r["duration_seconds"]
            for r in self.experiment_data
            if r["duration_seconds"] > 0
        ]
        avg_duration = sum(durations) / len(durations) if durations else 0

        return {
            "total_runs": total_runs,
            "successful_runs": successful_runs,
            "failed_runs": failed_runs,
            "timeout_runs": timeout_runs,
            "error_runs": error_runs,
            "success_rate": (
                (successful_runs / total_runs * 100) if total_runs > 0 else 0
            ),
            "average_duration": round(avg_duration, 2),
        }


def get_available_worlds():
    """
    获取所有可用的世界文件
    """
    if not os.path.exists(WORLDS_DIRECTORY):
        print(f"错误: 世界文件目录 '{WORLDS_DIRECTORY}' 不存在!")
        return []

    world_files = []
    for file in os.listdir(WORLDS_DIRECTORY):
        if file.endswith(".world"):
            world_files.append(os.path.join(WORLDS_DIRECTORY, file))

    if not world_files:
        print(f"警告: 在目录 '{WORLDS_DIRECTORY}' 中未找到任何 .world 文件!")

    return world_files


def generate_random_position():
    """
    生成随机的机器人起始位置
    """
    start_x = round(random.uniform(X_RANGE[0], X_RANGE[1]), 2)
    start_y = round(random.uniform(Y_RANGE[0], Y_RANGE[1]), 2)
    start_z = 0.1  # 一般保持在地面以上一点点
    start_yaw = round(random.uniform(YAW_RANGE[0], YAW_RANGE[1]), 2)

    return {"x": start_x, "y": start_y, "z": start_z, "yaw": start_yaw}


def generate_random_goal_position(start_pos):
    """
    生成随机的目标终点位置
    确保终点与起点距离合理（不太近也不太远）
    """
    # 最小和最大距离限制
    min_distance = 3.0  # 最小3米距离
    max_distance = 10.0  # 最大10米距离

    max_attempts = 50  # 最大尝试次数
    for attempt in range(max_attempts):
        goal_x = round(random.uniform(X_RANGE[0], X_RANGE[1]), 2)
        goal_y = round(random.uniform(Y_RANGE[0], Y_RANGE[1]), 2)

        # 计算与起点的距离
        distance = (
            (goal_x - start_pos["x"]) ** 2 + (goal_y - start_pos["y"]) ** 2
        ) ** 0.5

        if min_distance <= distance <= max_distance:
            return {"x": goal_x, "y": goal_y, "z": 0.1}

    # 如果没有找到合适的位置，生成一个固定偏移的位置
    goal_x = start_pos["x"] + 5.0  # 向X方向偏移5米
    goal_y = start_pos["y"] + 2.0  # 向Y方向偏移2米

    # 确保在有效范围内
    goal_x = max(X_RANGE[0], min(X_RANGE[1], goal_x))
    goal_y = max(Y_RANGE[0], min(Y_RANGE[1], goal_y))

    return {"x": round(goal_x, 2), "y": round(goal_y, 2), "z": 0.1}


def run_single_simulation(world_file, start_pos, goal_pos, run_number):
    """
    启动一次ROS仿真并等待它完成

    Args:
        world_file: 世界文件路径
        start_pos: 机器人起始位置字典 (包含 x, y, z, yaw)
        goal_pos: 机器人目标位置字典 (包含 x, y, z)
        run_number: 当前运行的编号

    Returns:
        dict: 包含仿真结果的详细信息
        {
            'success': bool,
            'status': str,
            'duration': float,
            'exit_code': int,
            'error_message': str,
            'path_data': list  # 新增：路径数据
        }
    """
    # 获取世界文件的绝对路径
    abs_world_path = os.path.abspath(world_file)

    # 构建 ros2 launch 命令，包含终点参数
    command = [
        "ros2",
        "launch",
        ROS_PACKAGE_NAME,
        ROS_LAUNCH_FILE,
        f"world:={abs_world_path}",
        f'start_x:={start_pos["x"]}',
        f'start_y:={start_pos["y"]}',
        f'start_z:={start_pos["z"]}',
        f'start_yaw:={start_pos["yaw"]}',
        f'goal_x:={goal_pos["x"]}',
        f'goal_y:={goal_pos["y"]}',
        f'goal_z:={goal_pos["z"]}',
        "headless:=true",  # <--- 新增：启用无头模式
    ]

    print(f">>> 第 {run_number} 次仿真启动命令:")
    print(f"    {' '.join(command)}")

    # 记录开始时间
    start_time = time.time()

    try:
        # 运行命令并等待其完成
        # 当您的算法节点退出，并导致整个launch进程结束时，这里才会继续往下走
        result = subprocess.run(
            command,
            check=False,  # 不要因为非零退出码而抛出异常
            text=True,
            capture_output=False,  # 让输出直接显示到终端
            timeout=300,  # 设置5分钟超时，防止仿真卡死
        )

        # 计算运行时间
        duration = time.time() - start_time

        if result.returncode == 0:
            print(f">>> 第 {run_number} 次仿真成功完成! (耗时: {duration:.2f}秒)")
            return {
                "success": True,
                "status": "success",
                "duration": duration,
                "exit_code": result.returncode,
                "error_message": None,
                "path_data": [],  # 将在未来版本中从实际路径文件中加载
            }
        else:
            print(
                f">>> 第 {run_number} 次仿真异常结束 (退出码: {result.returncode}, 耗时: {duration:.2f}秒)"
            )
            return {
                "success": False,
                "status": "failed",
                "duration": duration,
                "exit_code": result.returncode,
                "error_message": f"进程退出码: {result.returncode}",
                "path_data": [],
            }

    except subprocess.TimeoutExpired:
        duration = time.time() - start_time
        print(
            f">>> 第 {run_number} 次仿真超时 (超过 300 秒)，强制终止! (耗时: {duration:.2f}秒)"
        )
        return {
            "success": False,
            "status": "timeout",
            "duration": duration,
            "exit_code": None,
            "error_message": "仿真超时 (超过 300 秒)",
            "path_data": [],
        }
    except KeyboardInterrupt:
        print("\n>>> 用户中断实验!")
        raise
    except Exception as e:
        duration = time.time() - start_time
        print(f">>> 第 {run_number} 次仿真出现错误: {e} (耗时: {duration:.2f}秒)")
        return {
            "success": False,
            "status": "error",
            "duration": duration,
            "exit_code": None,
            "error_message": str(e),
            "path_data": [],
        }


def main():
    """
    主函数，循环执行仿真
    """
    print("=" * 80)
    print("🚀 开始批量仿真实验")
    print(f"📊 总计划运行次数: {NUM_RUNS}")
    print(f"🗺️  地图目录: {WORLDS_DIRECTORY}")
    print(f"📍 X坐标范围: {X_RANGE}")
    print(f"📍 Y坐标范围: {Y_RANGE}")
    print(f"📝 数据记录: {'启用' if ENABLE_LOGGING else '禁用'}")
    print("=" * 80)

    # 初始化数据记录器
    logger = ExperimentLogger()

    # 获取所有可用的世界文件
    available_worlds = get_available_worlds()

    if not available_worlds:
        print("❌ 没有找到可用的世界文件，实验终止!")
        return

    print(f"✅ 找到 {len(available_worlds)} 个世界文件:")
    for world in available_worlds:
        print(f"   - {os.path.basename(world)}")
    print()

    # 统计变量
    successful_runs = 0
    failed_runs = 0
    timeout_runs = 0
    error_runs = 0

    try:
        # 主循环：执行 NUM_RUNS 次仿真
        for i in range(1, NUM_RUNS + 1):
            print(f"\n{'='*20} 运行 {i}/{NUM_RUNS} {'='*20}")

            # 随机选择一个世界文件
            selected_world = random.choice(available_worlds)

            # 生成随机的起始位置
            start_position = generate_random_position()

            # 生成随机的终点位置
            goal_position = generate_random_goal_position(start_position)

            print(f"🗺️  选择的地图: {os.path.basename(selected_world)}")
            print(
                f"📍 起始位置: X={start_position['x']}, Y={start_position['y']}, Yaw={start_position['yaw']:.2f}"
            )
            print(f"🎯 目标位置: X={goal_position['x']}, Y={goal_position['y']}")

            # 运行单次仿真
            result = run_single_simulation(
                selected_world, start_position, goal_position, i
            )

            # 记录数据
            logger.log_simulation_run(
                run_number=i,
                world_file=selected_world,
                start_pos=start_position,
                goal_pos=goal_position,  # 新增：终点位置
                status=result["status"],
                duration=result["duration"],
                exit_code=result["exit_code"],
                error_message=result["error_message"],
                path_data=result.get("path_data", []),  # 新增：路径数据
            )

            # 更新统计
            if result["status"] == "success":
                successful_runs += 1
            elif result["status"] == "failed":
                failed_runs += 1
            elif result["status"] == "timeout":
                timeout_runs += 1
            elif result["status"] == "error":
                error_runs += 1

            print(
                f"📈 当前统计: 成功 {successful_runs} 次, 失败 {failed_runs} 次, 超时 {timeout_runs} 次, 错误 {error_runs} 次"
            )

            # 如果不是最后一次运行，就等待一段时间
            if i < NUM_RUNS:
                print(f"⏳ 等待 {SLEEP_BETWEEN_RUNS} 秒后开始下次仿真...")
                time.sleep(SLEEP_BETWEEN_RUNS)

    except KeyboardInterrupt:
        print("\n\n⚠️  实验被用户中断!")

    # 获取并打印最终统计
    total_runs = successful_runs + failed_runs + timeout_runs + error_runs
    stats = logger.get_summary_stats()

    print("\n" + "=" * 80)
    print("🎯 实验完成!")
    print(f"📊 总运行次数: {total_runs}")
    print(f"✅ 成功次数: {successful_runs}")
    print(f"❌ 失败次数: {failed_runs}")
    print(f"⏰ 超时次数: {timeout_runs}")
    print(f"🚫 错误次数: {error_runs}")

    if total_runs > 0:
        success_rate = (successful_runs / total_runs) * 100
        print(f"📈 成功率: {success_rate:.1f}%")

    if stats and "average_duration" in stats:
        print(f"⏱️  平均运行时间: {stats['average_duration']:.2f} 秒")

    if ENABLE_LOGGING and logger.csv_file_path:
        print("📄 数据已保存到:")
        print(f"   CSV: {logger.csv_file_path}")
        print(f"   JSON: {logger.json_file_path}")

    print("=" * 80)


if __name__ == "__main__":
    main()
