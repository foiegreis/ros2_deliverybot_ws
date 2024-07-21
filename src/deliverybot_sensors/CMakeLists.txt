cmake_minimum_required(VERSION 3.8)
project(deliverybot_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(pcl_conversions REQUIRED)  # Corrected from pcl_conversion to pcl_conversions
#find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

#C++ --------------------------------------------------------

#add_executable(lidar_fusion_node src/lidar_fusion_node.cpp)
#ament_target_dependencies(lidar_fusion_node
 #rclcpp
  #sensor_msgs
 # pcl_conversions
  #PCL
  #tf2_ros
  #tf2_sensor_msgs
#)

#install(TARGETS
 # lidar_fusion_node
 # DESTINATION lib/${PROJECT_NAME}
#)


#Py -----------------------------------------------------------

ament_python_install_package(${PROJECT_NAME})

#install python executables
install(PROGRAMS
  deliverybot_sensors/lidar_fusion_node.py


  DESTINATION lib/${PROJECT_NAME})

#--------------------------------------------------------------------
# export dependencies
ament_export_dependencies(${dependencies})
ament_package()
