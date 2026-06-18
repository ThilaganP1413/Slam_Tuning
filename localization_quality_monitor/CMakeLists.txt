cmake_minimum_required(VERSION 3.8)
project(localization_quality_monitor)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

add_compile_options(-O3 -Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(Eigen3 REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LocalizationScore.msg"
  "msg/LocalizationScoreMetrics.msg"
)

add_executable(localization_quality_monitor_node
  src/localization_quality_monitor.cpp
)

ament_target_dependencies(
  localization_quality_monitor_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  cv_bridge
)

target_include_directories(localization_quality_monitor_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

if(TARGET Eigen3::Eigen)
  target_link_libraries(localization_quality_monitor_node Eigen3::Eigen)
else()
  target_include_directories(localization_quality_monitor_node PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
  )
endif()

target_link_libraries(localization_quality_monitor_node
  ${OpenCV_LIBS}
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(
  localization_quality_monitor_node
  "${cpp_typesupport_target}"
)

install(TARGETS
  localization_quality_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()