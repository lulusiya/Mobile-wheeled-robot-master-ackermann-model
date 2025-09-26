#!/bin/bash
# 修正版路径跟踪测试脚本 - 按照路径点 (0,0) → (0,1) → (1,1) → (1,2) → (2,3) 移动
# 重要改进：转向时保持最小线速度，避免原地转向
# 注意：此脚本仅用于简单测试，实际应用请使用PathFollower节点

# 停止脚本时确保机器人停止
trap 'echo "Stopping robot..."; ros2 topic pub /ackermann_steering_controller/reference_unstamped geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -1; exit' SIGINT SIGTERM

# 配置参数（可根据实际机器人调整）
MIN_LINEAR_SPEED=0.60  # 转向时的最小线速度 (m/s)
NORMAL_LINEAR_SPEED=0.40  # 正常移动速度 (m/s)
ANGULAR_SPEED=1.2       # 角速度 (rad/s)
TURNING_FACTOR=1.3      # 转向时间修正系数（考虑边移动边转向）

echo "Starting path test: (0,0) → (0,1) → (1,1) → (1,2) → (2,3)"
echo "Using turning strategy: moving while turning (min speed = ${MIN_LINEAR_SPEED}m/s)"

# 1. 初始定位（假设机器人在(0,0)朝向+y方向）
echo "Initial position assumed at (0,0) facing +Y"
sleep 1

# 2. 从 (0,0) 移动到 (0,1) [直线前进]
echo "Moving from (0,0) to (0,1)..."
ros2 topic pub /ackermann_steering_controller/reference_unstamped geometry_msgs/Twist \
  "{linear: {x: ${NORMAL_LINEAR_SPEED}}, angular: {z: 0.0}}" -1
sleep 0.3  # 以0.4m/s速度前进1m需要2.5秒

ros2 topic pub /ackermann_steering_controller/reference_unstamped geometry_msgs/Twist \
  "{linear: {x: ${NORMAL_LINEAR_SPEED}}, angular: {z: 0.0}}" -1
sleep 0.3  # 以0.4m/s速度前进1m需要2.5秒

# 3. 从 (0,1) 转向到 (1,1) 方向 [右转90°]
echo "Turning right 90° to face (1,1) while moving..."
ros2 topic pub /ackermann_steering_controller/reference_unstamped geometry_msgs/Twist \
  "{linear: {x: ${MIN_LINEAR_SPEED}}, angular: {z: -${ANGULAR_SPEED}}}" -1
sleep $(echo "0.5 * ${TURNING_FACTOR}" | bc -l)  # 转向90°(1.57rad)需要更长时间

ros2 topic pub /ackermann_steering_controller/reference_unstamped geometry_msgs/Twist \
  "{linear: {x: ${MIN_LINEAR_SPEED}}, angular: {z: -${ANGULAR_SPEED}}}" -1
sleep $(echo "0.5 * ${TURNING_FACTOR}" | bc -l)  # 转向90°(1.57rad)需要更长时间

# 4. 从 (0,1) 移动到 (1,1) [直线前进]
echo "Moving from (0,1) to (1,1)..."
ros2 topic pub /ackermann_steering_controller/reference_unstamped geometry_msgs/Twist \
  "{linear: {x: ${NORMAL_LINEAR_SPEED}}, angular: {z: 0.0}}" -1
sleep 0.5  # 前进1m

ros2 topic pub /ackermann_steering_controller/reference_unstamped geometry_msgs/Twist \
  "{linear: {x: ${NORMAL_LINEAR_SPEED}}, angular: {z: 0.0}}" -1
sleep 0.5  # 前进1m

# 5. 从 (1,1) 转向到 (1,2) 方向 [左转90°]
echo "Turning left 90° to face (1,2) while moving..."
ros2 topic pub /ackermann_steering_controller/reference_unstamped geometry_msgs/Twist \
  "{linear: {x: ${MIN_LINEAR_SPEED}}, angular: {z: ${ANGULAR_SPEED}}}" -1
sleep $(echo "5 * ${TURNING_FACTOR}" | bc -l)  # 转向90°

# 6. 从 (1,1) 移动到 (1,2) [直线前进]
echo "Moving from (1,1) to (1,2)..."
ros2 topic pub /ackermann_steering_controller/reference_unstamped geometry_msgs/Twist \
  "{linear: {x: ${NORMAL_LINEAR_SPEED}}, angular: {z: 0.0}}" -1
sleep 5  # 前进1m

# 7. 从 (1,2) 转向到 (2,3) 方向 [左转45°]
echo "Turning left 45° to face (2,3) while moving..."
ros2 topic pub /ackermann_steering_controller/reference_unstamped geometry_msgs/Twist \
  "{linear: {x: ${MIN_LINEAR_SPEED}}, angular: {z: ${ANGULAR_SPEED}}}" -1
sleep $(echo "0.7 * ${TURNING_FACTOR}" | bc -l)  # 转向45°需要更长时间

# 8. 从 (1,2) 移动到 (2,3) [斜向前进]
echo "Moving from (1,2) to (2,3)..."
ros2 topic pub /ackermann_steering_controller/reference_unstamped geometry_msgs/Twist \
  "{linear: {x: $(echo "${NORMAL_LINEAR_SPEED} * 0.85" | bc -l)}, angular: {z: 0.0}}" -1
sleep 4.2  # 前进√2≈1.414m，速度稍降，增加时间确保到达

# 9. 完成路径，停止
echo "Path completed! Stopping robot..."
ros2 topic pub /ackermann_steering_controller/reference_unstamped geometry_msgs/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" -1
sleep 0.5

echo "Path test completed successfully!"
