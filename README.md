# LKA-Lane-Keeping-Assist
Hardware-in-the-Loop LKA system using STM32 and OpenCV
# Lane Keeping Assist (LKA) - Hardware-in-the-Loop & Computer Vision 

This repository contains the perception and embedded control implementation of a Lane Keeping Assist (LKA) system. It demonstrates a Hardware-in-the-Loop (HIL) architecture bridging real-time Computer Vision (Python/OpenCV) with bare-metal embedded programming (STM32).

##  System Architecture

The project is divided into two main components:
1. **Computer Vision (Python):** Processes dynamic driving video streams, detects lane lines using Canny edge detection and Hough transforms, and calculates the trajectory offset and required steering angle.
2. **Embedded Controller (STM32 Nucleo):** Receives the steering data via asynchronous UART, processes the control logic in C (bare-metal), and ensures actuator safety.

##  Technologies & Hardware
* **Hardware:** STM32 Nucleo Board (e.g., Nucleo-F4xx)
* **Software/Languages:** Python 3, C (Bare-metal)
* **Libraries:** OpenCV, NumPy
* **Communication:** UART (Serial)

##  Key Features
* **Real-time Lane Detection:** Optimized OpenCV pipeline for fast frame processing.
* **HIL Serial Communication:** Continuous data transmission between the PC and the STM32 board.
* **Bare-Metal Programming:** Direct register manipulation for UART configuration and NVIC interrupt handling.
* **Failsafe Mechanism:** Implementation of a software Watchdog (via SysTick) to safely halt the system in case of communication loss.
