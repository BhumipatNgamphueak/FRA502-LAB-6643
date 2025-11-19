cmake_minimum_required(VERSION 3.8)
project(lab4_controller_rviz)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################ FIND DEPENDENCIES FIRST #################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # ⭐ MUST BE BEFORE rosidl_generate_interfaces
##########################################################

################ GENERATE SERVICES AFTER FINDING PACKAGES #################
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputeIK.srv"
  "srv/GetTargetPose.srv"
  "srv/SetControlMode.srv"
  DEPENDENCIES geometry_msgs
)
###########################################################################

################ INSTALL PYTHON SCRIPTS ##############
install(PROGRAMS
  scripts/mock_robot_node.py
  scripts/controller_node_rviz.py
  scripts/teleop_jog_keyboard.py
  scripts/workspace_analyzer_node.py
  scripts/robot_utils.py
  scripts/end_effector_publisher.py
  scripts/random_pose_node.py
  DESTINATION lib/${PROJECT_NAME}
)
######################################################

################ INSTALL LAUNCH FILES ################
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
######################################################

################ TESTING #############################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
######################################################

ament_package()