cmake_minimum_required(VERSION 3.5.0)

project(ldm_fleet_server_ros2 LANGUAGES C CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake QUIET)

if (ament_cmake_FOUND)
  find_package(builtin_interfaces REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(ldm_fleet_msgs REQUIRED)
  find_package(ldm_fleet REQUIRED)
  find_package(Eigen3 REQUIRED)

  add_executable(ldm_fleet_server_ros2
    src/main.cpp
    src/utilities.cpp
    src/ServerNode.cpp
    src/ServerNodeConfig.cpp
  )
  target_link_libraries(ldm_fleet_server_ros2
    ${ldm_fleet_LIBRARIES}
    Eigen3::Eigen
  )
  target_include_directories(ldm_fleet_server_ros2
    PRIVATE
      ${ldm_fleet_INCLUDE_DIRS}
  )
  ament_target_dependencies(ldm_fleet_server_ros2
    rclcpp
    ldm_fleet_msgs
  )

  
  install(
    TARGETS ldm_fleet_server_ros2
    RUNTIME DESTINATION lib/ldm_fleet_server_ros2
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )

  ament_package()

else()
  message("ament_cmake not found so skipping this ROS2 package")
endif()

# -----------------------------------------------------------------------------


# -----------------------------------------------------------------------------


