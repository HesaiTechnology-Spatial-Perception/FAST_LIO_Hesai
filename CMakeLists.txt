cmake_minimum_required(VERSION 3.8)
project(fast_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")

  # Keep the historical conservative default, but allow high-core hosts to
  # opt in explicitly when scan-matching throughput is the bottleneck.
  set(FAST_LIO_MP_PROC_NUM "" CACHE STRING
      "OpenMP workers for FAST-LIO scan matching (default: 3 on x86 hosts)")
  if(FAST_LIO_MP_PROC_NUM STREQUAL "")
    if(N GREATER 4)
      set(FAST_LIO_MP_PROC_NUM 3)
    elseif(N GREATER 3)
      set(FAST_LIO_MP_PROC_NUM 2)
    else()
      set(FAST_LIO_MP_PROC_NUM 1)
    endif()
  endif()
else()
  set(FAST_LIO_MP_PROC_NUM 1 CACHE STRING
      "OpenMP workers for FAST-LIO scan matching")
endif()

if(NOT FAST_LIO_MP_PROC_NUM MATCHES "^[1-9][0-9]*$")
  message(FATAL_ERROR "FAST_LIO_MP_PROC_NUM must be a positive integer")
endif()
if(FAST_LIO_MP_PROC_NUM GREATER 1)
  add_definitions(-DMP_EN)
endif()
add_definitions(-DMP_PROC_NUM=${FAST_LIO_MP_PROC_NUM})
message("OpenMP workers for scan matching: ${FAST_LIO_MP_PROC_NUM}")

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")


# ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  pcl_ros
  pcl_conversions
)

# Thirdparty libraries
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

message(Eigen: ${EIGEN3_INCLUDE_DIR})
message(STATUS "PCL: ${PCL_INCLUDE_DIRS}")

set(msg_files
  "msg/Pose6D.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(fastlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
target_include_directories(fastlio_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(fastlio_mapping ${PCL_LIBRARIES} Eigen3::Eigen)

list(APPEND EOL_LIST "foxy" "galactic" "eloquent" "dashing" "crystal")

if($ENV{ROS_DISTRO} IN_LIST EOL_LIST)
  # Custommsg to support foxy & galactic
  rosidl_target_interfaces(fastlio_mapping
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(fastlio_mapping ${cpp_typesupport_target})
endif()

ament_target_dependencies(fastlio_mapping ${dependencies})

option(FAST_LIO_PREPROCESS_ASAN_TEST "Build the preprocessing bounds regression with AddressSanitizer" OFF)
if(FAST_LIO_PREPROCESS_ASAN_TEST)
  add_executable(fastlio_preprocess_test tools/test_preprocess_bounds.cpp src/preprocess.cpp)
  target_include_directories(fastlio_preprocess_test PRIVATE src include ${PCL_INCLUDE_DIRS})
  target_compile_options(fastlio_preprocess_test PRIVATE -fsanitize=address -fno-omit-frame-pointer -O1 -g)
  target_link_libraries(fastlio_preprocess_test ${PCL_LIBRARIES} -fsanitize=address)
  target_compile_definitions(fastlio_preprocess_test PRIVATE FAST_LIO_TEST_ROS2)
  ament_target_dependencies(fastlio_preprocess_test rclcpp sensor_msgs pcl_conversions)
endif()

# ---------------- Install --------------- #
install(TARGETS fastlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  tools/run_fastlio.sh
  tools/prepare_runtime_config.py
  tools/mapping_control.py
  tools/launch_process.py
  tools/bag_metadata.py
  tools/check_input.py
  tools/check_config.py
  tools/check_map.py
  tools/pcap_to_rosbag/pcap_to_rosbag_ros1.sh
  tools/pcap_to_rosbag/pcap_to_rosbag_ros2.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  tools/pcap_to_rosbag/common.sh
  tools/process_control.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install the public model catalog explicitly; local overrides are not distributable.
install(FILES
  config/jt16.yaml
  config/jt32.yaml
  config/jt64p.yaml
  config/jt128.yaml
  config/MT60.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES
  launch/mapping_jt16.launch.py
  launch/mapping_jt32.launch.py
  launch/mapping_jt64p.launch.py
  launch/mapping_jt128.launch.py
  launch/mapping_mt60.launch.py
  launch/mapping.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
