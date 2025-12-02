# ESP32-RL-Crawler  
**Fully Autonomous On-Board Q-Learning – 2-Servo Crawling Robot with Ultrasonic Reward & Swarm-Ready OTA**

[![Platform](https://img.shields.io/badge/Platform-ESP32%20DevKit-brightgreen)](https://espressif.com)
[![RL](https://img.shields.io/badge/RL-Tabular%20Q--Learning-blue)](#)
[![WiFi/OTA](https://img.shields.io/badge/Features-WiFi%20AP%20+%20OTA-orange)](#)
[![Swarm](https://img.shields.io/badge/Swarm-Up%20to%208%20robots-9cf)](#)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Stars](https://img.shields.io/github/stars/aibgr/ESP32-RL-Crawler?style=social)](https://github.com/aibgr/ESP32-RL-Crawler/stargazers)

> **Zero off-board computation** – Complete Q-Learning (35×35 table) runs on ESP32 at 100–200 Hz  
> Real-time reward directly from HC-SR04 ultrasonic sensor  
> Supports up to 8 robots simultaneously with unique WiFi AP + OTA hostname  
> Full source code + schematics included

### Key Features
- 35 discrete posture states + 35 possible actions → 1225 Q-values stored in RAM
- Distance-based reward: `r = 2.5 × Δdistance` (encourages forward motion)
- ε-greedy with decay (0.8 → 0.1) → converges in ~10 episodes (~2–3 min)
- Smooth servo trajectories with configurable speed
- Startup health check (ultrasonic + servo reset
- I2C 16×2 LCD real-time feedback
- EEPROM-persisted robot ID (1–8) → unique SSID: `ESP32-AP-1` … `ESP32-AP-8`
- Non-blocking OTA updates via FreeRTOS task

### Core Q-Learning Update Rule (executed every step on ESP32)
<img src="https://github.com/user-attachments/assets/6cdee654-7b21-434c-b0a6-4700784bfc67" alt="Q-Learning formula"/>

### Say Hi!

<table>
  <tr>
    <td align="center"><img src="https://github.com/user-attachments/assets/4a897820-342e-4c58-95cb-eb8b307d94eb" alt="Robot prototype"/></td>
  </tr>
  <tr>

  </tr>
</table>

<p align="center">
  <em>2-servo crawling robot learning optimal forward gait completely on-board using only an ultrasonic sensor as reward signal</em>
</p>

### Hardware
- ESP32 Dev Module
- 2× SG90/MG90S servo motors
- HC-SR04 ultrasonic sensor
- 16×2 I2C LCD
- Custom 3D-printed linkage mechanism
