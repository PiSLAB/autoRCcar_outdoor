cmake_minimum_required(VERSION 3.8)
project(autorccar_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(autorccar_interfaces REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
)
link_directories(${Eigen3_INCLUDE_DIRS})

# build ekf-ros
add_executable(gnss_ins
  src/main_ekf.cpp 
  src/ekf_ros_wrapper.cpp
  src/ekf.cpp
  src/ins_toolbox.cpp
)
ament_target_dependencies(gnss_ins
  rclcpp 
  std_msgs 
  nav_msgs 
  autorccar_interfaces
  Eigen3
)
target_link_libraries(gnss_ins
  yaml-cpp
)
install(TARGETS
  gnss_ins
  DESTINATION lib/${PROJECT_NAME}
)

# build example (without ROS)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/example)
add_executable(test_ekf 
  example/test_ekf.cpp 
  src/ekf.cpp
  src/ins_toolbox.cpp
)
target_link_libraries(test_ekf
  yaml-cpp
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
