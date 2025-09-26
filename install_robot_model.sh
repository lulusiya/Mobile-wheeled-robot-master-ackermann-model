#!/bin/bash
set -e

# 1. 定义路径
PKG_PATH=~/Mobile-wheeled-robot-master-ackermann-model/src/simple_robot
URDF_FILE=$PKG_PATH/urdf/robot.urdf.xacro
GAZEBO_MODEL_PATH=~/.gazebo/models/simple_robot

echo "==> 开始安装 simple_robot 模型到 Gazebo ..."

# 2. 确保目录存在
mkdir -p $GAZEBO_MODEL_PATH

# 3. 生成临时 urdf 文件
echo "==> 使用 xacro 生成 URDF ..."
ros2 run xacro xacro $URDF_FILE > /tmp/robot.urdf

# 4. 转换为 SDF
echo "==> 使用 gz sdf 转换为 SDF ..."
gz sdf -p /tmp/robot.urdf > $GAZEBO_MODEL_PATH/model.sdf

# 5. 写入 model.config
echo "==> 写入 model.config ..."
cat > $GAZEBO_MODEL_PATH/model.config <<EOF
<?xml version="1.0"?>
<model>
  <name>simple_robot</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>you@example.com</email>
  </author>
  <description>
    Simple Ackermann robot for Gazebo
  </description>
</model>
EOF

echo "✅ 模型已安装到: $GAZEBO_MODEL_PATH"
echo "现在你可以在 .world 文件里用 <uri>model://simple_robot</uri> 来加载它啦！"
