# Radar System using Arduino, Ultrasonic Sensor, and Processing

## Overview
This project implements a radar system using an Arduino Uno, an ultrasonic sensor, and a servo motor. The system scans its surroundings in real-time and visualizes detected objects on a radar-like interface built with Processing. It demonstrates how embedded hardware and software visualization can work together for object detection and distance measurement.

---

## Features
- Real-time object detection and visualization
- Servo-based scanning mechanism
- Distance measurement using an ultrasonic sensor
- Graphical radar interface using Processing
- Adjustable scanning angles and detection range

---

## Components Required
- Arduino Uno (or compatible board)
- Ultrasonic Sensor (HC-SR04)
- Servo Motor (SG90 or similar)
- Breadboard and jumper wires
- USB cable for Arduino
- Computer with Arduino IDE and Processing IDE

---

## Software Requirements
- Arduino IDE (for programming Arduino)
- Processing IDE (for visualization interface)
- Serial communication enabled between Arduino and Processing

---

## Setup Instructions

### Hardware Setup
1. Connect the ultrasonic sensor to the Arduino:
   - VCC → 5V  
   - GND → GND  
   - TRIG → Digital Pin 9  
   - ECHO → Digital Pin 10  
2. Connect the servo motor:
   - VCC → 5V  
   - GND → GND  
   - Signal → Digital Pin 11  
3. Ensure all components share a common ground.

### Software Setup
1. Open the Arduino IDE and upload the Arduino sketch to your board.  
2. Install Processing IDE on your computer.  
3. Open the Processing sketch provided in the repository.  
4. Update the serial port in the Processing sketch to match your Arduino’s COM port (check Arduino IDE > Tools > Port).  
5. Run the Processing sketch to start the radar visualization.

---

## How It Works
1. The servo motor rotates from 0° to 180° in defined steps.  
2. At each angle, the ultrasonic sensor measures the distance to the nearest object.  
3. Distance and angle values are sent via Serial to the Processing interface.  
4. Processing displays the radar-like visualization, plotting detected objects based on angle and distance.  

---
## Demo
You can watch the working demo of the project here:  
[LinkedIn Project Demo](https://www.linkedin.com/posts/manas-chawla-277155325_exploring-hardware-from-the-ground-up-over-activity-7346979436623245312-DxYu?utm_source=social_share_send&utm_medium=member_desktop_web&rcm=ACoAAE5AzuQB8uUn1Qxu8AhspvGezZdzuErUFA4)  


---

## Applications
- Object detection and tracking  
- Robotics navigation systems  
- Educational projects for learning embedded systems and visualization  
- Proof of concept for low-cost radar-like systems  

---

## Future Improvements
- Replace ultrasonic sensor with LiDAR for better accuracy  
- Add multiple sensors for 360° scanning  
- Implement data logging and analytics  
- Wireless communication between Arduino and visualization software  

---

## License
This project is licensed under the MIT License. See the LICENSE file for details.

---
