cmake_minimum_required(VERSION 3.5)
project(rosslt_lib)

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
find_package(rosslt_msgs REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

include_directories(include)

# Build library
add_library(rosslt SHARED
    src/trackingnode.cpp)

ament_target_dependencies(rosslt rclcpp rosslt_msgs rosidl_typesupport_introspection_cpp)
ament_export_include_directories(include)

ament_export_interfaces(export_rosslt HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

# Install library
install(TARGETS
  rosslt
  EXPORT export_rosslt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
