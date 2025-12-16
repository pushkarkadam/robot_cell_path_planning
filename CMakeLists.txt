cmake_minimum_required(VERSION 3.8)
project(robot_cell_path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(point_move src/point_move.cpp)
ament_target_dependencies(
  point_move
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2_geometry_msgs"
  "geometry_msgs"
  "moveit_msgs"
  "moveit_visual_tools"
)

add_executable(calibration_path src/calibration_path.cpp src/read_data.cpp)
ament_target_dependencies(
  calibration_path
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2_geometry_msgs"
  "geometry_msgs"
  "moveit_msgs"
  "moveit_visual_tools"
)

add_executable(weld_path src/weld_path.cpp src/read_data.cpp)
ament_target_dependencies(
  weld_path
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2_geometry_msgs"
  "geometry_msgs"
  "moveit_msgs"
  "moveit_visual_tools"
)

add_executable(hello_yaml src/hello_yaml.cpp)
ament_target_dependencies(
  hello_yaml
  "yaml_cpp_vendor"
)

add_executable(read_transformation src/read_transformation.cpp)
ament_target_dependencies(
  read_transformation
  "yaml_cpp_vendor"
  "Eigen3"
)

add_executable(read_viapoints src/read_viapoints.cpp src/read_data.cpp)
ament_target_dependencies(
  read_viapoints 
)

install(TARGETS point_move
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS calibration_path
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS weld_path
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS hello_yaml
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS read_transformation
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS read_viapoints
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Let the compiler search for headers in the include folder
include_directories(include)

# define a library target called read_data
add_library(read_data src/read_data.cpp)

# this line to export the library
ament_export_targets(read_data HAS_LIBRARY_TARGET)

# install the include/robot_cell_path_planning
install(
  DIRECTORY include/robot_cell_path_planning
  DESTINATION include
)

install(
  TARGETS read_data
  EXPORT read_data
  LIBRARY DESTINATION lib 
  ARCHIVE DESTINATION lib 
  RUNTIME DESTINATION bin 
  INCLUDES DESTINATION include
)

ament_package()
