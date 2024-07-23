# ros2_deliverybot_ws

### ROS2 DeliveryBot: Autonomous Delivery 
Welcome to the ROS2 DeliveryBot project
I'm developing an autonomous Ackermann steering-controlled delivery robot. All 3D models are custom-made by the author.


### DeliveryBot

This project's goal is to create an autonomous (and possibly the cutest) 4 wheeled Ackermann Steering DeliveryBot that aims to navigate from a parking lot to another goal position, avoiding fixed and moving obstacles. Also, it will be able to perform non holonomic parking.

Final goal is to spawn multiple DeliveryBots and perform fleet management.


<img src="imgs/deliverybot.png" alt="DeliveryBot">
<img src="imgs/deliverybot2.png" alt="DeliveryBot in ParkingLot world">

#### ParkingLot world
In this project we use a self-made world depicting a Parking Lot.

<img src="imgs/parkinglot.png" alt="DeliveryBot in ParkingLot world">



#### What's ready:
- Robot Urdf description
- Ackermann steering control
- Joystick teleoperation 
- Imu integration
- 3D lidars integration (4 lidars, that get fused in a single pointcloud - deliverybot_sensors/lidar_fusion_node.py)
- Depth camera integration
- Twistmux for input management. Joystick and Keyboard inputs available
- Twistmux safety stop (danger zone) and speed decrease (warning zone)


#### What's coming:
- Improved odometry
- SLAM implementation (Localization and Mapping)
- Navigation
- Additional navigation input to TwistMux
- Computer vision for mobile obstacle and parking lines detection
- Path Planning
- Behavioral Planning

#### Commits History
- first commit: description, ackermann control, teleoperation
- second commit: lidars, depth camera
- third commit: twistmux and safety stop

#### Snapshots
Lidar pointcloud and dept camera
<img src="imgs/lidar_depth_camera.png" alt="DeliveryBot in ParkingLot world">

Speed decrease if object is detected in warning zone distance. Rviz Markers visualization.
<img src="imgs/warning_zone.png" alt="DeliveryBot in ParkingLot world">

Twistmux Safety Stop if object is detected in danger zone distance. Rviz Markers visualization.
<img src="imgs/stop_zone.png" alt="DeliveryBot in ParkingLot world">
### Build the workspace
```
cd ros2_deliverybot_ws
colcon build
source ./install/setup.bash
```

### Launch the latest implementation
```
(empty world)
ros2 launch deliverybot_bringup deliverybot.launch.py

(ParkingLot world)
ros2 launch deliverybot_bringup deliverybot.launch.py world_name:=ParkingLot

```

### Acknowledgment

This project is part of my portfolio. You can check out more of my work at [Robotics Unveiled / Greta Russi Portfolio ](https://www.roboticsunveiled.com/portfolio/).

<img src="imgs/roboticsunveiled.png" alt="Roboticsunveiled">

