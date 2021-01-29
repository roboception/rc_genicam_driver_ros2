cmake_minimum_required(VERSION 3.5)
project(rc_genicam_driver)

include(cmake/project_version.cmake)

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
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(rc_common_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(image_transport REQUIRED)
find_package(RC_GENICAM_API 2.4.4 REQUIRED)

set(NODE_NAME rc_genicam_driver)
set(LIB_NAME ${NODE_NAME}_core)

set(DEPS
  rclcpp
  rclcpp_components
  sensor_msgs
  stereo_msgs
  rc_common_msgs
  diagnostic_updater
  image_transport
)

include_directories(
  ${RC_GENICAM_API_INCLUDE_DIRS}
)

add_library(${LIB_NAME} SHARED
  src/genicam_driver.cpp
  src/publishers/camera_info_publisher.cpp
  src/publishers/camera_param_publisher.cpp
  src/publishers/confidence_publisher.cpp
  src/publishers/depth_publisher.cpp
  src/publishers/disparity_color_publisher.cpp
  src/publishers/disparity_publisher.cpp
  src/publishers/error_depth_publisher.cpp
  src/publishers/error_disparity_publisher.cpp
  src/publishers/image_publisher.cpp
  src/publishers/points2_publisher.cpp
)

ament_target_dependencies(${LIB_NAME}
  ${DEPS}
)

target_link_libraries(${LIB_NAME}
  rc_genicam_api::rc_genicam_api
)

add_executable(${NODE_NAME}
  src/genicam_driver_main.cpp
)

target_link_libraries(${NODE_NAME} ${LIB_NAME})

rclcpp_components_register_nodes(${LIB_NAME} "rc::GenICamDriver")
set(node_plugins "${node_plugins}rc::GenICamDriver;$<TARGET_FILE:rc_genicam_driver>\n")

install(TARGETS ${NODE_NAME} ${LIB_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#install(DIRECTORY params
#  DESTINATION share/${PROJECT_NAME}
#)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(${LIB_NAME})
ament_export_dependencies(${DEPS})
ament_package()

######################################
## Define information for packaging ##
######################################
# optionally specify dependencies of the debian package here (comma separated!)
set(CPACK_DEBIAN_PACKAGE_DEPENDS "ros-$ENV{ROS_DISTRO}-rc-common-msgs, rc-genicam-api (>= 2.4.4) | ros-$ENV{ROS_DISTRO}-rc-genicam-api (>= 2.4.4)")
include(cmake/package_debian.cmake)
