cmake_minimum_required(VERSION 3.8)
project(deliverybot_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

install(DIRECTORY 
  launch 
  config
  DESTINATION share/${PROJECT_NAME}
)

#C++ -----------------------------------------------------
add_executable(twistmux_speed_monitoring src/twistmux_speed_monitoring.cpp)
ament_target_dependencies(twistmux_speed_monitoring rclcpp rclcpp_action std_msgs sensor_msgs visualization_msgs tf2_ros pcl_conversions PCL)
target_link_libraries(twistmux_speed_monitoring ${PCL_LIBRARIES})

install(TARGETS

  twistmux_speed_monitoring

  DESTINATION lib/${PROJECT_NAME})

#Python ---------------------------------------------------
ament_python_install_package(${PROJECT_NAME})

#install python executables
install(PROGRAMS

  deliverybot_teleop/twist_to_twist_stamped.py
  deliverybot_teleop/twist_to_twist_stamped_and_safety.py

  DESTINATION lib/${PROJECT_NAME})

#--------------------------------------------------------------------
# export dependencies
ament_export_dependencies(${dependencies})

ament_package()
