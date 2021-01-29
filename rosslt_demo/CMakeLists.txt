cmake_minimum_required(VERSION 3.5)
project(rosslt_demo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosslt_msgs REQUIRED)
find_package(rosslt_lib REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# Build nodes
add_executable(slt_talker src/slt_talker.cpp)
ament_target_dependencies(slt_talker rclcpp std_msgs rosslt_msgs rosslt_lib)
target_link_libraries(slt_talker rosslt_lib::rosslt)

add_executable(slt_listener src/slt_listener.cpp)
ament_target_dependencies(slt_listener rclcpp std_msgs rosslt_msgs rosslt_lib)
target_link_libraries(slt_listener rosslt_lib::rosslt)

add_executable(vis_talker src/visualization_talker.cpp)
ament_target_dependencies(vis_talker rclcpp std_msgs visualization_msgs rosslt_msgs rosslt_lib)
target_link_libraries(vis_talker rosslt_lib::rosslt)

add_executable(vis_listener src/visualization_listener.cpp)
ament_target_dependencies(vis_listener rclcpp std_msgs visualization_msgs rosslt_msgs rosslt_lib interactive_markers)
target_link_libraries(vis_listener rosslt_lib::rosslt)

add_executable(slt_trajectory src/slt_trajectory.cpp)
ament_target_dependencies(slt_trajectory rclcpp std_msgs visualization_msgs rosslt_msgs rosslt_lib tf2_ros tf2 tf2_geometry_msgs)
target_link_libraries(slt_trajectory rosslt_lib::rosslt)

add_executable(slt_quad_node src/slt_quad_node.cpp)
ament_target_dependencies(slt_quad_node rclcpp std_msgs visualization_msgs rosslt_msgs rosslt_lib interactive_markers tf2_ros tf2 tf2_geometry_msgs)
target_link_libraries(slt_quad_node rosslt_lib::rosslt)

add_executable(slt_message_statistics src/slt_message_statistics.cpp)
ament_target_dependencies(slt_message_statistics rclcpp std_msgs visualization_msgs rosslt_msgs)

add_executable(expression_benchmark src/expression_benchmark.cpp)
ament_target_dependencies(expression_benchmark rosslt_lib)
target_link_libraries(expression_benchmark rosslt_lib::rosslt)

# Install nodes
install(TARGETS
  slt_talker
  slt_listener
  slt_trajectory
  slt_quad_node
  vis_talker
  vis_listener
  slt_message_statistics
  expression_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY meshes
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
