cmake_minimum_required(VERSION 3.8)
project(my_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Install directories
install(DIRECTORY 
  config
  launch
  description
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/set_initial_pose.py
  scripts/activate_nav2_nodes.py
  scripts/diagnose_bt_navigator.py
  scripts/get_bt_navigator_errors.py
  scripts/activate_nav2_nodes.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
