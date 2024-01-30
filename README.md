# Autonomous Drone Project

[**Here drone can be seen following the target autnomously**](https://drive.google.com/file/d/1POETLoX9we2UhCyZxAZZfYm8ndg1grND/view?usp=drive_link)

![Drone Image](Drone1.jpeg)

## Overview

This project implements an autonomous drone using **Ardupilot**, **Dronekit (Python)**, and a **Jetson Nano** equipped with a **YOLO (You Only Look Once)** model for target detection. The drone's objective is to detect multiple targets on the ground from a height of 30 meters, capture images of the targets, and autonomously drop a payload on a specified drop zone.

## Features

- **Target Detection:** Utilizes a YOLO model for real-time target detection.
- **Autonomous Flight:** Uses Ardupilot and Dronekit to achieve autonomous flight capabilities.
- **Payload Delivery:** Drops payload on the designated drop zone.
- **Image Capture:** Takes pictures of detected targets during the mission.

## Hardware Setup

- **Drone Frame:** *Custom Manufactured (3D Printed)*
- **Flight Controller:** *Pixhawk 2.4.8 *
- **Onboard Computer:** Jetson Nano
- **Camera:** *Usb Webcam 720p*
- **Payload Mechanism:** *Servo Based Mechanism*

## Software Dependencies

- **Ardupilot**
- **Dronekit**
- **Jetson Nano libraries**
- **YOLO model for target detection**
- *Any other relevant software dependencies*

