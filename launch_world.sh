#!/bin/bash

# 四轮机器人仿真场景启动脚本

echo "=== 四轮机器人仿真场景选择 ==="
echo "请选择要启动的场景："
echo ""
echo " 1. empty - 空旷世界（默认）"
echo " 2. big_maze - 大迷宫"
echo " 3. huge_obstacle - 中心巨大障碍物"
echo " 4. static_obstacles - 静态障碍物"
echo " 5. dynamic_obstacles - 动态障碍物"
echo " 6. warehouse - 仓库环境"
echo " 7. office - 办公室环境"
echo " 8. urban_street - 城市街道"
echo " 9. factory - 工厂环境"
echo "10. parking_lot - 停车场"
echo "11. hospital - 医院环境"
echo "12. construction_site - 建筑工地"
echo "13. garden_maze - 花园迷宫"
echo "14. race_track - 赛道"
echo "15. farm - 农场"
echo "16. library - 图书馆"
echo "17. restaurant - 餐厅"
echo "18. shopping_mall - 购物中心"
echo "19. airport - 机场"
echo "20. obstacle_course - 障碍训练场"
echo "21. moving_platforms - 移动平台"
echo "22. narrow_corridors - 狭窄走廊"
echo "23. multi_level - 多层环境"
echo "24. random_obstacles - 随机障碍物"
echo "25. dense_forest - 密集森林"
echo "26. testing_ground - 测试场地"
echo "27. test - 动态测试环境（红色箱子+蓝色圆柱体）"
echo ""

# 获取用户输入
read -p "请输入场景编号 (1-27): " choice

# 设置默认值
world_file="empty.world"

# 根据选择设置世界文件
case $choice in
    1) world_file="empty.world" ;;
    2) world_file="big_maze.world" ;;
    3) world_file="huge_obstacle.world" ;;
    4) world_file="static_obstacles.world" ;;
    5) world_file="dynamic_obstacles.world" ;;
    6) world_file="warehouse.world" ;;
    7) world_file="office.world" ;;
    8) world_file="urban_street.world" ;;
    9) world_file="factory.world" ;;
    10) world_file="parking_lot.world" ;;
    11) world_file="hospital.world" ;;
    12) world_file="construction_site.world" ;;
    13) world_file="garden_maze.world" ;;
    14) world_file="race_track.world" ;;
    15) world_file="farm.world" ;;
    16) world_file="library.world" ;;
    17) world_file="restaurant.world" ;;
    18) world_file="shopping_mall.world" ;;
    19) world_file="airport.world" ;;
    20) world_file="obstacle_course.world" ;;
    21) world_file="moving_platforms.world" ;;
    22) world_file="narrow_corridors.world" ;;
    23) world_file="multi_level.world" ;;
    24) world_file="random_obstacles.world" ;;
    25) world_file="dense_forest.world" ;;
    26) world_file="testing_ground.world" ;;
    27) world_file="test.world" ;;
    *) 
        echo "无效选择，使用默认的空旷世界"
        world_file="empty.world"
        ;;
esac

# 启动仿真
echo ""
echo "正在启动场景: $world_file"
echo ""
ros2 launch simple_robot robot.launch.py world:="$(pwd)/src/simple_robot/worlds/$world_file"

