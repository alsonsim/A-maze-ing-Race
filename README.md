# A-MAZE-ING RACE – CG1111A Project

## Overview
This project was developed as part of **CG1111A – Engineering Principles and Practice I** at NUS.  
We built an autonomous mBot capable of navigating a maze, detecting colored strips, and reacting with programmed behaviors. The bot integrates multiple subsystems (lane correction, color detection, movement control) into a complete navigation solution.

## Features
- **Lane Correction** – Proportional-Derivative (PD) control using ultrasonic and IR sensors for wall-following stability  
- **Black Strip Detection** – Makeblock line sensor triggers color reading sequences  
- **Color Detection** – Custom RGB LED + LDR circuit with Euclidean distance algorithm for reliable color identification  
- **Movement & Turning** – Calibrated forward, left, right, double turns, and U-turns for consistent navigation  
- **Celebratory Tune** – Plays the Mario “Stage Win” theme at the maze’s end 🎵  

## Tech & Tools
- **Hardware:** mBot, Ultrasonic Sensor, IR Sensor, LDR, RGB LEDs, 2-to-4 Decoder, Breadboard  
- **Software:** Arduino (C/C++), PD control logic, Euclidean distance-based color classification  
- **Skills Applied:** Embedded systems, circuit design, sensor integration, control algorithms, teamwork  

## Setup
1. Open the Arduino IDE and load the source code:  
   ```bash
   B03_S4_T4_Source_Code.ino
2. Connect your mBot via USB and select the correct board/port.  
3. Upload the code to the mBot.  
4. Ensure all circuits (ultrasonic, IR, RGB LED, LDR) are wired as per report diagrams.  
5. Place the mBot at the maze’s start line and power it on.  

## Team Members
- **Sim Chin Kai Alson** – Ultrasonic sensor circuit, code development, skirting design  
- **Tan Feng Yuan** – Skirting, celebratory tune code, documentation  
- **Then Yi Ting Noelle** – Color sensor & IR circuit, color detection, movement algorithm  

## Reflection
This project combined hardware and software into a cohesive embedded system.  
We overcame challenges in sensor calibration, circuit debugging, and consistent turning.  
Despite setbacks, the final robot successfully demonstrated autonomous maze navigation,  
reinforcing our understanding of engineering design and teamwork.
