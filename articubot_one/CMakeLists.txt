cmake_minimum_required(VERSION 3.5)
project(articubot_one)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  articubot_one/scripts/way_points.py
  articubot_one/scripts/waypoints_commander.py


  DESTINATION lib/${PROJECT_NAME}
)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(realsense_coordinate_converter src/realsense_coordinate_converter.cpp)
ament_target_dependencies(realsense_coordinate_converter rclcpp sensor_msgs)

add_executable(calculate_path src/calculate_path.cpp)
ament_target_dependencies(calculate_path rclcpp sensor_msgs geometry_msgs PCL pcl_conversions)

add_executable(calculate_path_dev_reverse src/calculate_path_dev_reverse.cpp)
ament_target_dependencies(calculate_path_dev_reverse rclcpp sensor_msgs geometry_msgs PCL pcl_conversions)

add_executable(get_laser_range src/get_laser_range.cpp)
ament_target_dependencies(get_laser_range rclcpp sensor_msgs)

add_executable(follow_cooridor_srv src/follow_cooridor_srv.cpp)
ament_target_dependencies(follow_cooridor_srv rclcpp sensor_msgs geometry_msgs PCL pcl_conversions tf2_ros)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
  follow_cooridor_srv
  realsense_coordinate_converter
  calculate_path
  calculate_path_dev_reverse
  get_laser_range
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
