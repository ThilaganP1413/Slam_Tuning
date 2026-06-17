cmake_minimum_required(VERSION 3.8)
project(localization_quality_monitor)

# --- Optimization: Release build with explicit -O3 -----------------------
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

add_compile_options(-O3 -Wall -Wextra -Wpedantic)

# --- Dependencies ----------------------------------------------------------
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

# --- Executable --------------------------------------------------------
add_executable(localization_quality_monitor
  src/localization_quality_monitor.cpp
)

ament_target_dependencies(localization_quality_monitor
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  cv_bridge
  OpenCV
)

# Eigen3 is header-only; link the imported interface target if available,
# otherwise fall back to the include-dirs variable for older CMake configs.
if(TARGET Eigen3::Eigen)
  target_link_libraries(localization_quality_monitor Eigen3::Eigen)
else()
  target_include_directories(localization_quality_monitor PRIVATE ${EIGEN3_INCLUDE_DIRS})
endif()

target_link_libraries(localization_quality_monitor ${OpenCV_LIBS})

# --- Install -------------------------------------------------------------
install(TARGETS
  localization_quality_monitor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The default copyright/cpplint checks are skipped here since this is a
  # single-author internal tool; remove the two lines below if you want
  # full ament linting enforced in CI.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()