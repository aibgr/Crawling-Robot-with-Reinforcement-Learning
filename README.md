# Q-Crawler  
**Fully On-Board Tabular Q-Learning on Arduino – 2-Armed Crawling Robot with Real-Time Condition Monitoring**  
100% learning and decision-making runs on an 8-bit microcontroller (no PC, no ROS, no external processing)

[![Arduino](https://img.shields.io/badge/Platform-Arduino%20Uno%20%2F%20Mega-brightgreen)](https://www.arduino.cc)
[![Algorithm](https://img.shields.io/badge/RL-Tabular%20Q--Learning-blue)](#)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![GitHub stars](https://img.shields.io/github/stars/aibgr/Q-Crawler?style=social)](https://github.com/aibgr/Q-Crawler/stargazers)

### Overview
A 2-armed crawling robot that autonomously discovers optimal locomotion gaits using **pure on-board Q-Learning** running directly on Arduino Uno/Mega.

- State space: 8 discrete arm configurations  
- Action space: 6 primitive movements  
- Reward: forward progress (+10), energy penalty (–1), stall/overheat (–50)  
- Real-time condition monitoring: battery voltage, motor current, temperature, stall detection

### Core Q-Learning Update Rule (executed every loop on Arduino)
<img src="https://github.com/user-attachments/assets/6cdee654-7b21-434c-b0a6-4700784bfc67" alt="Q-Learning update rule"/>

### Say Hi!

<table>
  <tr>
    <td align="center"><img src="https://github.com/user-attachments/assets/4a897820-342e-4c58-95cb-eb8b307d94eb" alt="Robot prototype"/></td>
  </tr>
</table>

<p align="center">
  <em>3D-printed 2-armed crawler successfully learning optimal gait in real-time on Arduino</em>
</p>

### Key Achievements
- One of the first fully embedded tabular Q-Learning implementations for locomotion on an 8-bit AVR microcontroller 
- Converges to optimal gait in ~80–120 episodes (~4–6 minutes real-time)  
- Complete RL loop runs at 100–200 Hz using < 12 KB flash memory  
- Zero external RL libraries – pure hand-written C++  
- Built-in safety system with automatic motor shutdown

### Repository Contents
