cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

# Automatyczne znajdowanie zależności i ustawianie środowiska
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Opcje kompilacji
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Znajdź dodatkowe zależności
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Dodaj węzeł "robot_grasping_task"
add_executable(robot_grasping_task src/robot_grasping_task.cpp)
ament_target_dependencies(robot_grasping_task
    "rclcpp"
    "moveit_ros_planning_interface"
    "tf2_ros"
    "tf2_geometry_msgs"
)

# Instalacja pliku wykonywalnego robot_grasping_task
install(TARGETS robot_grasping_task DESTINATION lib/${PROJECT_NAME})

# Dodaj węzeł "hello_moveit"
add_executable(hello_moveit src/hello_moveit.cpp)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(hello_moveit
    "rclcpp"
    "moveit_ros_planning_interface"
    "moveit_visual_tools"
)

find_package(gazebo_msgs REQUIRED)
add_executable(one_grasp src/one_grasp.cpp)
ament_target_dependencies(one_grasp
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "gazebo_msgs"
)
install(TARGETS one_grasp DESTINATION lib/${PROJECT_NAME})

add_executable(pick_and_place src/pick_and_place.cpp)
ament_target_dependencies(pick_and_place
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "gazebo_msgs"
)
install(TARGETS pick_and_place DESTINATION lib/${PROJECT_NAME})

add_executable(show_sides_grasps src/show_sides_grasps.cpp)
ament_target_dependencies(show_sides_grasps
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "gazebo_msgs"
)
install(TARGETS show_sides_grasps DESTINATION lib/${PROJECT_NAME})

# Instalacja pliku wykonywalnego hello_moveit
install(TARGETS hello_moveit DESTINATION lib/${PROJECT_NAME})

# Testy i linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Automatyczna instalacja dodatkowych zasobów
ament_auto_package(INSTALL_TO_SHARE launch worlds maps config models)