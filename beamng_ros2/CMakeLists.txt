cmake_minimum_required(VERSION 3.8)
project(beamng_ros2)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -fvisibility=hidden
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(beamng_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

set(BEAMNG_LIBRARY ros2_beamng)

add_library(${BEAMNG_LIBRARY} SHARED
  src/init.cpp
  src/beamng_node.cpp
  src/vehicle_node.cpp
  src/sensor_registry.cpp
  src/sensor_factories.cpp
  src/shared_memory_reader.cpp
)
target_include_directories(${BEAMNG_LIBRARY} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${BEAMNG_LIBRARY} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_definitions(${BEAMNG_LIBRARY} PUBLIC
  BNG_ROS2_COMPILING
)
ament_target_dependencies(${BEAMNG_LIBRARY}
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  beamng_msgs
  sensor_msgs
  rosidl_typesupport_introspection_cpp
  rosidl_typesupport_cpp
)

install(TARGETS ${BEAMNG_LIBRARY}
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

ament_package()
