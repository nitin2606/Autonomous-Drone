# Autonomous Drone Project

![Drone Image](Drone1.jpeg)

## Overview

This project implements an autonomous drone using **Ardupilot**, **Dronekit (Python)**, and a **Jetson Nano** equipped with a **YOLO (You Only Look Once)** model for target detection. The drone's objective is to detect multiple targets on the ground from a height of 30 meters, capture images of the targets, and autonomously drop a payload on a specified drop zone.

## Features

- **Target Detection:** Utilizes a YOLO model for real-time target detection.
- **Autonomous Flight:** Uses Ardupilot and Dronekit to achieve autonomous flight capabilities.
- **Payload Delivery:** Drops payload on the designated drop zone.
- **Image Capture:** Takes pictures of detected targets during the mission.

## Hardware Setup

- **Drone Frame:** *Specify the drone frame model*
- **Flight Controller:** *Specify the flight controller model*
- **Onboard Computer:** Jetson Nano
- **Camera:** *Specify the camera model*
- **Payload Mechanism:** *Specify the payload release mechanism*

## Software Dependencies

- **Ardupilot**
- **Dronekit**
- **Jetson Nano libraries**
- **YOLO model for target detection**
- *Any other relevant software dependencies*

## Getting Started

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/your-username/Autonomous-Drone.git
   cd Autonomous-Drone
