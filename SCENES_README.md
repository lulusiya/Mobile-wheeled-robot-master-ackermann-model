# 四轮机器人仿真场景

本项目包含了26个不同的Gazebo仿真场景，用于测试四轮移动机器人的导航和控制算法。

## 场景列表

### 基础场景
1. **empty.world** - 空旷世界，基础测试环境
2. **big_maze.world** - 大迷宫，路径规划测试
3. **huge_obstacle.world** - 中心巨大障碍物，绕行测试
4. **static_obstacles.world** - 静态障碍物，避障测试
5. **dynamic_obstacles.world** - 动态障碍物，动态避障测试

### 室内环境
6. **warehouse.world** - 仓库环境，物流机器人测试
7. **office.world** - 办公室环境，服务机器人测试
8. **hospital.world** - 医院环境，医疗机器人测试
9. **library.world** - 图书馆环境，安静环境导航
10. **restaurant.world** - 餐厅环境，服务机器人测试
11. **shopping_mall.world** - 购物中心，复杂室内环境
12. **airport.world** - 机场环境，大型公共空间

### 室外环境
13. **urban_street.world** - 城市街道，户外导航测试
14. **construction_site.world** - 建筑工地，复杂地形测试
15. **garden_maze.world** - 花园迷宫，自然环境导航
16. **farm.world** - 农场环境，农业机器人测试
17. **dense_forest.world** - 密集森林，复杂自然环境

### 特殊场景
18. **race_track.world** - 赛道，高速导航测试
19. **parking_lot.world** - 停车场，精确导航测试
20. **factory.world** - 工厂环境，工业机器人测试

### 挑战性场景
21. **obstacle_course.world** - 障碍训练场，综合技能测试
22. **moving_platforms.world** - 移动平台，动态环境导航
23. **narrow_corridors.world** - 狭窄走廊，精确控制测试
24. **multi_level.world** - 多层环境，3D导航测试
25. **random_obstacles.world** - 随机障碍物，不规则环境测试
26. **testing_ground.world** - 测试场地，算法验证环境

## 使用方法

### 方法1：使用便捷脚本（推荐）
```bash
# 进入项目目录
cd /Users/mortyang/Desktop/Works/RU-ITMO/four_wheeled_robot

# 运行启动脚本
./launch_world.sh
```

然后根据提示选择想要的场景编号即可。

### 方法2：直接使用ROS2命令
```bash
# 启动特定场景
ros2 launch four_wheeled_robot launch_sim.launch.py world:=$(ros2 pkg prefix four_wheeled_robot)/share/four_wheeled_robot/worlds/WORLD_FILE_NAME.world

# 例如启动大迷宫场景
ros2 launch four_wheeled_robot launch_sim.launch.py world:=$(ros2 pkg prefix four_wheeled_robot)/share/four_wheeled_robot/worlds/big_maze.world
```

### 方法3：使用相对路径
```bash
# 从项目根目录运行
ros2 launch four_wheeled_robot launch_sim.launch.py world:=/Users/mortyang/Desktop/Works/RU-ITMO/four_wheeled_robot/worlds/WORLD_FILE_NAME.world
```

## 场景特点

### 导航挑战等级
- **初级**: empty, static_obstacles, warehouse, office
- **中级**: big_maze, garden_maze, hospital, library, restaurant
- **高级**: dynamic_obstacles, moving_platforms, narrow_corridors, obstacle_course
- **专家级**: multi_level, dense_forest, random_obstacles

### 环境类型
- **结构化环境**: warehouse, office, hospital, library, shopping_mall, airport
- **半结构化环境**: urban_street, construction_site, farm, parking_lot, factory
- **非结构化环境**: dense_forest, random_obstacles, garden_maze
- **动态环境**: dynamic_obstacles, moving_platforms

### 测试用途
- **路径规划**: big_maze, garden_maze, narrow_corridors
- **避障算法**: static_obstacles, dynamic_obstacles, random_obstacles
- **精确控制**: parking_lot, narrow_corridors, obstacle_course
- **多传感器融合**: dense_forest, multi_level, construction_site
- **实时决策**: moving_platforms, dynamic_obstacles, race_track

## 技术说明

所有场景都使用SDF格式定义，兼容Gazebo仿真器。每个场景都包含：
- 基础地面平面
- 太阳光照
- 相应的静态/动态障碍物
- 适当的材质和颜色标识

动态障碍物使用Gazebo的脚本功能实现，包含轨迹动画和循环运动。

## 扩展说明

您可以通过修改SDF文件来自定义场景，或者创建新的世界文件。每个场景都是模块化设计，便于修改和扩展。

## 注意事项

1. 确保已正确安装ROS2和Gazebo
2. 确保四轮机器人包已正确编译
3. 动态障碍物场景可能需要更多计算资源
4. 某些复杂场景可能需要调整机器人的启动位置

## 故障排除

如果遇到世界文件无法加载的问题：
1. 检查文件路径是否正确
2. 确认SDF文件格式无误
3. 检查Gazebo版本兼容性
4. 查看终端输出的错误信息

