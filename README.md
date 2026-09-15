# stm32-ball-balancing-control-On-a-Moving-Vehicle-
An STM32-based embedded motion-control system for vehicle-mounted ball balancing, featuring PID control, feedforward compensation, signal filtering, and dual-MCU architecture.


# STM32 Ball Balancing Control System

> An embedded motion-control system developed for the 2026 TI Cup National Undergraduate Electronic Design Competition, Problem H.

This project implements a vehicle-mounted ball balancing system based on **two STM32F103C8T6 microcontrollers**.

The system integrates **line following, differential-drive motion control, ball position stabilization, PID control, feedforward compensation, signal filtering, and real-time embedded programming**.

Unlike a software-only simulation, this project was developed as a complete physical system, covering:

* PCB design
* Hardware assembly
* Low-level peripheral development
* Sensor acquisition and signal processing
* Motion control
* PID controller design
* Feedforward compensation
* Embedded software architecture
* System integration
* Experimental debugging and parameter tuning

The project received a **Third Prize** in the competition.

However, the main purpose of this repository is not to document the competition result, but to preserve the **engineering process, control strategies, implementation details, and lessons learned from building the system from the ground up**.

## Technical Highlights

* Dual STM32F103C8T6 architecture
* STM32 Standard Peripheral Library
* Nine-channel line following
* Differential-drive control
* Ball-and-beam stabilization
* Position-form PID control
* Integral dead-zone
* First-order low-pass filtering
* Acceleration feedforward compensation
* Differential-motion compensation
* 20 ms ball-control loop
* Mode-based embedded software architecture
* Custom PCB design

## Engineering Perspective

A central lesson from this project is that a controller cannot be considered independently from the physical system.

Actuator travel, mechanical friction, sensor characteristics, computational resources, sampling periods, and system dynamics all impose constraints on achievable control performance.

Therefore, this repository focuses not only on **what controller was implemented**, but also on **why it was designed this way, what assumptions were made, and where the physical system ultimately became the limiting factor**.

> **Do not only ask whether a system works. Ask why it works, how it fails, and whether it can be understood and repaired when it does.**
