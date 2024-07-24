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
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)


include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

#C++ --------------------------------------------------------

#add_executable(pointcloud_to_laserscan_node src/pointcloud_to_laserscan_node.cpp)
#ament_target_dependencies(pointcloud_to_laserscan_node rclcpp sensor_msgs pcl_conversions PCL)
#target_link_libraries(pointcloud_to_laserscan_node ${PCL_LIBRARIES})


install(TARGETS
  #pointcloud_to_laserscan_node
  DESTINATION lib/${PROJECT_NAME}
 )


#Py -----------------------------------------------------------

ament_python_install_package(${PROJECT_NAME})

#install python executables
install(PROGRAMS
  deliverybot_sensors/pointcloud_fusion_node.py


  DESTINATION lib/${PROJECT_NAME})

#--------------------------------------------------------------------
# export dependencies
ament_export_dependencies(${dependencies})
ament_package()
