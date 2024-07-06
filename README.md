# **AI.SCARA** – Open-source low-cost SCARA robot 
## Abstract
The aim of this project was to design and build a low-cost robot based on the industrial concept SCARA, which meets the requirements in terms of rigidity and repeatability for application in a wide range of manipulation tasks primarily implemented in a university environment. The project includes not only the mechanical design, but also design and implementation of the electrical circuitry, control system, and design of a human-machine graphical interface for basic controland diagnosis of the robotic structure.
<p align="center"><img src="docs/AISCARAfullrender.png" height="400"></p>

## Repository Index
* [AISCARA_ControlSystem](https://github.com/Katzoun/AI_SCARA/tree/main/AISCARA_ControlSystem) - Source codes for AISCARA control system
* [AISCARA_GUI](https://github.com/Katzoun/AI_SCARA/tree/main/AISCARA_GUI) - Source codes for AISCARA user interface
* [docs](https://github.com/Katzoun/AI_SCARA/tree/main/docs) - Misc documentation files 
* [stl]() - STL files of robot parts

## Mechanical design


## Electrical design
Stepper motors with encoders were chosen to drive the robot. The main advantage of these motors is their very low purchase price compared to equally powerful servo drives. By controlling stepper motors in a closed loop, better dynamic characteristics can be achieved compared to open-loop stepper motors. Another indisputable advantage of closed-loop control is the possibility of position correction in case of overload. Reference position detection is provided by optical limit switches.

The robot controller (shown below) brings together all the electronic components necessary for the proper functioning of the SCARA robot - i.e. stepper motor controllers, microcontrollers that run the control system, power supplies and many others. Power is provided by three DC power supplies - LRS-350-36 (CL57 stepper drivers), LRS-100-12 (pneumatic equipment, DM320T driver, internal air compressors etc.) and RS-25-5 (MCUs)

There are two microcontrollers in the SCARA robot controller. The main MCU is the Teensy 4.1, which performs all the kinematic calculations for the robot's movements.
The reason for choosing the Teensy 4.1 microcontroller for robot motion control was mainly due to its architecture, which is compatible with user-friendly Arduino microcontrollers. The control of the robot peripherals is provided by Arduino Nano. 

<p align="center"><img src="docs/controller.png" height="400"></p>
<p align="center"><img src="docs/schematic controller.png" height="500"></p>
<p align="center"><img src="docs/schematic robot.png" height="348"></p>
