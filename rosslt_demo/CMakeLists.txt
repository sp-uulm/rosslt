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
find_package(rosslt_msgs REQUIRED)
find_package(rosslt_lib REQUIRED)

include_directories(include)

# Build nodes
add_executable(slt_talker src/slt_talker.cpp)
ament_target_dependencies(slt_talker rclcpp std_msgs rosslt_msgs rosslt_lib)
target_link_libraries(slt_talker rosslt_lib::rosslt)

add_executable(slt_listener src/slt_listener.cpp)
ament_target_dependencies(slt_listener rclcpp std_msgs rosslt_msgs rosslt_lib)
target_link_libraries(slt_listener rosslt_lib::rosslt)

# Install nodes
install(TARGETS
  slt_talker
  slt_listener
  DESTINATION lib/${PROJECT_NAME}
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