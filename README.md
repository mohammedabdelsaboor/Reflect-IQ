# Reflect IQ

Reflect IQ is a ROS 2-based intelligent rear-view monitoring system designed to enhance automotive safety. The system utilizes dual cameras and radar sensors to detect fast-approaching vehicles and initiate autonomous navigation responses, such as safe lane changes, to avoid potential collisions.

## Overview

Reflect IQ integrates computer vision and sensor fusion to continuously monitor rear-side traffic and assess surrounding risk. The system is built on ROS 2 for modularity and real-time performance.

## Hardware Components

- Raspberry Pi 4 Model B
- Two USB cameras (mounted on rear-right and rear-left of the vehicle)
- mmWave radar sensor (optional for velocity estimation)
- Power supply and necessary peripherals

## Software Components

- ROS 2 
- OpenCV for image processing
- YOLOv5 for object detection
- Custom ROS 2 nodes for:
  - Camera input
  - Radar data reading
  - Vehicle detection
  - Navigation and control logic

## System Features

- Real-time detection of fast-approaching vehicles from rear sides
- Time-to-collision estimation
- Autonomous lane-change recommendation or execution
- Modular ROS 2 architecture for scalability

## System Architecture

1. **Camera Nodes**: Capture video streams from left and right rear cameras.
2. **Radar Node**: Reads and publishes data from radar sensor.
3. **Detection Node**: Processes image and radar data to detect approaching vehicles using YOLOv5.
4. **Navigation Node**: Calculates safe maneuver (e.g., lane change) and issues control commands.

