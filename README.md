# Autonomous Football Robot

Remote-controlled ESP32-based robot capable of autonomous ball transport, line detection, and shooting mechanism.

**Award:** 🥉 3rd Place, First Semester Project Competition

## Overview

This project was developed as part of the First Semester Project (Erstsemesterprojekt) at Hochschule Niederrhein. The robot can pick up and transport a ball, detect boundary lines, shoot the ball using a servo mechanism, and return to its starting position autonomously.

## Features

- **Bluetooth Remote Control:** Full joystick control via RemoteXY mobile app
- **Autonomous Line Detection:** QTR-1A sensors detect black lines and trigger automatic responses
- **Ball Shooting Mechanism:** Servo-actuated kicking system
- **Custom 3D-Printed Components:** Bumper and mechanical parts designed in Tinkercad

## Hardware Components

- **Microcontroller:** ESP32
- **Motors:** 2x DC motors with H-bridge motor driver
- **Servo:** SG90 micro servo motor
- **Sensors:** 2x QTR-1A reflectance sensors (front and rear)
- **Power:** USB powerbank with custom 3D-printed holder
- **Mechanical:** Custom 3D-printed bumper and servo mount

## Software Stack

- **Language:** C++ (Arduino framework)
- **IDE:** Arduino IDE
- **Libraries:**
  - `ESP32Servo` - Servo motor control
  - `RemoteXY` - Bluetooth remote control interface
  - `BLEDevice` - Bluetooth Low Energy communication

## Technical Details

### Line Detection
- Threshold value: 4000 (analog reading ~4095 on black surface)
- Automatic reverse when front sensor detects line
- Automatic forward when rear sensor detects line

### Remote Control
- Bluetooth name: "CyberTruckLite"
- Password protected: "2004"
- Joystick-based differential drive control
- Button-triggered shooting mechanism

### Shooting Mechanism
- Servo rotation: 0° to 90°
- Kick duration: 500ms
- Automatic return to home position

## Setup Instructions

### Prerequisites
1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Add ESP32 board support to Arduino IDE
3. Install required libraries via Library Manager:
   - RemoteXY
   - ESP32Servo

### Installation
1. Clone this repository
2. Open `teslaLite.cpp` in Arduino IDE
3. Select ESP32 board (Tools → Board → ESP32 Dev Module)
4. Upload to ESP32

### Usage
1. Power on the robot
2. Install RemoteXY app on your smartphone ([iOS](https://apps.apple.com/app/remotexy/id1024653009) / [Android](https://play.google.com/store/apps/details?id=com.shemansky.remotexy))
3. Connect to "CyberTruckLite" via Bluetooth
4. Enter password: "2004"
5. Use joystick to control movement and button to shoot

## Project Team

- Muhammad Bagier Alaydrus
- Eren Cicekli
- Mitansh Morwale
- Nursaktyo Suryowibowo

## Project Information

**Course:** Erstsemesterprojekt (First Semester Project)  
**Institution:** Hochschule Niederrhein, FB03  
**Date:** Winter Semester 2024/2025  
**Duration:** ~3 months  
**Result:** 3rd Place in project competition

## Documentation

Full project documentation (in German) is available in `documentation.pdf`, including:
- System architecture
- Circuit diagrams
- 3D design files
- Implementation challenges and solutions
- Testing and results

## License

This project was created for educational purposes as part of university coursework.

## Acknowledgments

- Hochschule Niederrhein for providing lab facilities and components
- Course instructors for guidance and support
