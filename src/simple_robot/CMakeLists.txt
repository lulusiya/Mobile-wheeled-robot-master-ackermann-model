cmake_minimum_required(VERSION 3.8)
project(simple_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf config worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/move_obstacles.py
  scripts/task_executor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

