# ME/CS 129: Experimental Robotics — Autonomous Robot Project

This repository contains our final project for **ME/CS 129: Experimental Robotics** at Caltech.  
The course focuses on the **experimental realization of robotic systems**, combining software infrastructure, real-time sensing, and autonomous behavior development using Python. Throughout the term, we progressively integrated sensing, perception, mapping, and planning capabilities on a physical mobile robot, culminating in a final **autonomous multi-robot challenge**.

---

##  Project Overview

Our team designed and built an autonomous mobile robot from scratch, capable of **navigating a course, detecting obstacles, and interacting with the environment**.  
The robot’s hardware and software were fully integrated through multi-threaded Python nodes running on a Raspberry Pi, supporting real-time sensor fusion, motion control, and behavior planning.

### Key Capabilities
- **Autonomous Navigation:**  
  Followed course boundaries using infrared (IR) line sensors and real-time control of dual drive motors.
- **Obstacle Avoidance:**  
  Utilized ultrasonic sensors to detect nearby objects and dynamically replan to prevent collisions.
- **Magnetometer Localization:**  
  Estimated orientation and heading to maintain consistent world-frame positioning.
- **Prize Collection Mechanism:**  
  Used an electromagnet actuator to pick up small metallic “prizes” placed around the arena.
- **High-Level Planning:**  
  Implemented **Dijkstra’s algorithm** for goal-directed navigation and a **modified exploration version** to visit unexplored regions efficiently.
- **Behavioral Layering:**  
  Combined multiple low-level and high-level behaviors (track following, exploration, prize pickup) through modular Python scripts and ROS2-style publishers/subscribers.

---

##  Hardware Architecture

| Component | Description |
|------------|-------------|
| **Microcontroller** | Raspberry Pi (Python control) |
| **Drive System** | Two DC motors with rear-wheel drive |
| **Steering** | Passive front ball joint for smooth turns |
| **Sensors** | 3× Ultrasonic rangefinders, IR line sensors, Magnetometer |
| **Actuator** | Electromagnet for prize pickup |
| **Power** | Onboard battery pack with regulated 5 V/12 V rails |

---

##  Software Architecture

- **Language:** Python 3  
- **Main Modules:**
  - `motor.py` — low-level PWM control for motor speeds
  - `sensors.py` — asynchronous data polling from IR sensors
  - `map.py` — Dijkstra-based path planner and exploration logic with mapping
  - `behaviors.py` — task coordinator for exploring, goal seeking, and prize pickup
  - `main.py` — system integration and control loop orchestration

- **Techniques Used:**
  - Multi-threading for concurrent motor and sensor processes  
  - Event detection in noisy sensor data  
  - State estimation and finite-state machine behavior control  
  - Path planning and map-based goal selection

---

##  Demonstration

In the final challenge, our robot successfully:
- Navigated a multi-intersection track autonomously
- Avoided dynamic obstacles using live sensor feedback
- Explored unmapped regions using our modified Dijkstra’s algorithm
- Collected prizes using the electromagnet actuator

---

##  Course Context

> *ME/CS 129: Experimental Robotics* explores hands-on robotics development through hardware integration, sensing, perception, and autonomous control.  
Assignments build up from polling and multi-threaded architectures to map-based planning strategies, culminating in a fully autonomous robot demonstration.

---

##  Team

**Steven Romero-Ruiz** and **Ben Hong**  
California Institute of Technology  
Spring 2025

---

