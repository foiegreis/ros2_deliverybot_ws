cmake_minimum_required(VERSION 3.8)
project(deliverybot_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)


install(DIRECTORY 
  launch 
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# C++ -----------------------------------------------------------------
#add_executable(ackermann_to_cmd_vel_node src/ackermann_to_cmd_vel.cpp)
#ament_target_dependencies(ackermann_to_cmd_vel_node rclcpp ackermann_msgs geometry_msgs)

#install(TARGETS
 # ackermann_to_cmd_vel_node
 # DESTINATION lib/${PROJECT_NAME}
#)

#Python ---------------------------------------------------
ament_python_install_package(${PROJECT_NAME})

#install python executables
install(PROGRAMS
  deliverybot_controllers/twist_keyboard_stamped.py
  deliverybot_controllers/twist_joystick_stamped.py


  DESTINATION lib/${PROJECT_NAME})

#--------------------------------------------------------------------
# export dependencies
ament_export_dependencies(${dependencies})



ament_package()
