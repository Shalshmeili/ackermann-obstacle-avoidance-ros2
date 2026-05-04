# Ackermann Drive Robot with Autonomous Obstacle Avoidance and Camera Integration using ROS2

This project presents a complete ROS2 and Gazebo simulation of an autonomous Ackermann-drive robot. The robot is designed to move like a car using front-wheel steering, detect obstacles using a front-facing LiDAR sensor, capture images using an onboard camera, and avoid obstacles while continuing forward.

The project was developed for the ROBO 202 Software Development for Robotics course.

## Project Overview

The robot is modeled using URDF and simulated in Gazebo. It includes:

- Main chassis
- Four cylindrical wheels
- Front steering links
- WALL-E-inspired head structure
- Front-facing LiDAR sensor
- Front-mounted camera
- Gazebo Ackermann drive plugin
- Gazebo LiDAR plugin
- Gazebo camera plugin
- Autonomous obstacle avoidance node
- Telemetry monitoring node

The robot moves forward autonomously without keyboard control. When the LiDAR detects an obstacle within the stopping distance, the robot stops, captures an image using the camera, then avoids the obstacle using Ackermann steering by moving forward through a turning arc.

## Main Features

- ROS2 and Gazebo simulation
- URDF robot modeling
- Ackermann-drive motion
- Autonomous forward movement
- LiDAR-based obstacle detection
- Camera image capture when an obstacle is detected
- Static obstacle world with colored cylindrical pillars
- Live telemetry monitoring
- Velocity command publishing using `/cmd_vel`
- LiDAR readings from `/scan`
- Camera images from `/camera/image_raw`

## Robot Behavior

The autonomous behavior follows this sequence:

1. The robot moves forward.
2. The front-facing LiDAR scans the area in front of the robot.
3. If an obstacle is detected within the required distance, the robot stops.
4. The onboard camera captures and saves an image.
5. The robot avoids the obstacle using Ackermann steering.
6. After completing the turn, the robot continues moving forward.

This behavior respects the Ackermann-drive constraint because the robot does not rotate in place. Instead, it turns while moving forward, similar to a car.
