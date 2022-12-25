cmake_minimum_required(VERSION 3.8)
project(autorccar_ubloxf9r)

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

include_directories(include)
add_library(${PROJECT_NAME}
	src/ublox_decode.cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)
find_package(autorccar_interfaces REQUIRED) 

add_executable(ubloxf9r src/sensor_data_pub.cpp)
ament_target_dependencies(ubloxf9r rclcpp std_msgs autorccar_interfaces) 
target_link_libraries(ubloxf9r Boost::boost ${CMAKE_THREAD_LIBS_INIT})

add_executable(rawdata_listener src/sensor_data_sub.cpp)
ament_target_dependencies(rawdata_listener rclcpp std_msgs autorccar_interfaces)

install(TARGETS
  ubloxf9r
  rawdata_listener
  DESTINATION lib/${PROJECT_NAME})

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
