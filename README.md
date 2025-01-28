# Visual-Inertial Odometry with Synthetic Data using MATLAB
## Overview
This project focuses on implementing a **Visual-Inertial Odometry (VIO)** system using synthetic data generated with MATLAB's **drivingScenario** tool. VIO is a critical technology for estimating the position and orientation (pose) of a moving object by fusing data from visual sensors (e.g., a monocular camera) and inertial sensors (e.g., an IMU). The project leverages MATLAB's **insfilterErrorState** function to fuse data from the IMU and monocular camera, enabling robust and accurate pose estimation.

## Importance of Visual-Inertial Odometry
Visual-Inertial Odometry is a key component in many autonomous systems, such as drones, robots, and self-driving cars. It combines the strengths of visual and inertial sensors:
- **Visual sensors** provide rich environmental information but can struggle in low-texture or high-speed scenarios.
- **Inertial sensors** (IMUs) offer high-frequency motion data but suffer from drift over time.

By fusing these two data sources, VIO achieves higher accuracy and robustness, making it suitable for real-world applications where reliability is crucial.

## Key Features
- **Synthetic Data Generation**: The project uses MATLAB's **drivingScenario** tool to create realistic synthetic environments and sensor data, including camera images and IMU measurements.
- **Sensor Fusion**: The **insfilterErrorState** function is employed to fuse data from the monocular camera and IMU, ensuring accurate pose estimation.
- **Error-State Kalman Filter**: The **insfilterErrorState** implements an error-state Kalman filter, which is well-suited for handling the nonlinearities and uncertainties inherent in VIO systems.

## Applications
This project demonstrates the potential of VIO in applications such as:
- Autonomous navigation for drones and robots.
- Augmented and virtual reality systems.
- Localization and mapping in GPS-denied environments.

## Requirements
- MATLAB (with the Sensor Fusion and Tracking Toolbox).
- Basic understanding of sensor fusion, computer vision, and inertial navigation.

## Acknowledgments
This project utilizes MATLAB's powerful tools for sensor fusion and synthetic data generation, enabling the exploration of Visual-Inertial Odometry in a controlled and repeatable environment.



