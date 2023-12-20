# ESP32 Robot Control Algorithm

This repository contains the code for controlling a robot using an ESP32 microcontroller. The robot is equipped with various sensors and motors for movement.

## Prerequisites

Before using the code, make sure you have the following libraries installed:

- WiFi.h
- esp_now.h
- WiFiUdp.h

Additionally, the code utilizes the `vive510` library, so make sure to include it in your project.

## Hardware Setup

The robot's hardware includes ultrasonic sensors, motors, and Vive trackers. Connect the sensors and motors to the specified GPIO pins on the ESP32.

## Code Structure

The main code file is `Main.ino`, which includes the following functionalities:

- **Motor Control**: Controls the movement of the robot using PWM signals for different directions.
- **Ultrasonic Sensor**: Uses an ultrasonic sensor to measure distances and make decisions based on obstacles.
- **Vive Tracker Integration**: Interacts with Vive trackers for tracking position and orientation.

## Usage

1. Upload the code to your ESP32 board using the Arduino IDE.
2. Ensure that the required libraries are installed.
3. Power on the robot and monitor the serial output for debugging.

## Configuration

- WiFi credentials (`ssid` and `password`) should be configured in the code.
- Adjust GPIO pin assignments for sensors and motors based on your hardware setup.

## Modes of Operation

The robot has different modes of operation, including manual control, autonomous roaming, and special modes like fake and police modes.

- **Manual Mode**: Allows manual control of the robot using sliders.
- **Autonomous Mode**: The robot autonomously navigates based on obstacle detection.
- **Fake Mode**: Simulates movement based on predefined conditions.
- **Police Mode**: Implements specific movements based on Vive tracker positions.
