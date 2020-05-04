cmake_minimum_required(VERSION 3.5)
project(m5stack_example)

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(m5stack_msgs REQUIRED)

add_executable(plot_example src/plot_example.cpp)
ament_target_dependencies(plot_example rclcpp std_msgs m5stack_msgs)
target_include_directories(plot_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS plot_example
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

add_executable(stamp_example src/stamp_example.cpp)
ament_target_dependencies(stamp_example rclcpp std_msgs m5stack_msgs)
target_include_directories(stamp_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS stamp_example
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
