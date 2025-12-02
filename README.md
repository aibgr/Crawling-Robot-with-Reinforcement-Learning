# ESP32-RL-Crawler  
**Fully On-Board Q-Learning on ESP32 – 2-DOF Crawling Robot with Ultrasonic Reward & Epsilon-Greedy Exploration**

[![Platform](https://img.shields.io/badge/Platform-ESP32-brightgreen)](https://www.espressif.com)
[![RL](https://img.shields.io/badge/RL-Tabular%20Q--Learning-blue)](#)
[![WiFi/OTA](https://img.shields.io/badge/Features-WiFi%20AP%20%2B%20OTA-orange)](#)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Stars](https://img.shields.io/github/stars/aibgr/ESP32-RL-Crawler?style=social)](https://github.com/aibgr/ESP32-RL-Crawler/stargazers)

> **100% autonomous learning on microcontroller** – No PC, no ROS, no cloud  
> Real-time reward from **ultrasonic sensor** (forward progress = positive reward)  
> Full over-the-air (OTA) updates + unique AP per robot (supports swarm of 8 robots)

### Project Overview
A 2-servo crawling robot that **learns to move forward** using **tabular Q-Learning running entirely on ESP32**, with:
- 35 discrete posture states (servo combinations)
- Direct distance-based reward using HC-SR04 ultrasonic sensor
- ε-greedy action selection with decay
- Smooth servo interpolation
- Multi-robot support (1–8) with unique WiFi AP + OTA hostname
- LCD feedback + full health check on boot

### Core Q-Learning (Running Live on ESP32)
```text
Q(s,a) ← Q(s,a) + α [ r + γ max Q(s',a') − Q(s,a) ]
