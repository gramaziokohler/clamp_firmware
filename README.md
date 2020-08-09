# clamp_firmware
Firmware for embedded Arduino on remote-controlled actuators (clamps).

This repo is part of the [Robotic Assembled Timber Structures with Integral Timber Joints](https://github.com/gramaziokohler/integral_timber_joints) project. 

## Repo folder structure

- **/clamp_controller** - Contains one Visual Studio solutions that contains all clamp controller projects. Sub folder contains different VC++ projects for various controller.
- **/serial_radio** - Contains one Visual Studio solution for the Serial Radio. Sub folder contains different VC++ projects for various dongle firmware.
- **/libraries** - stores all the internal and external libraries for the c++ compiler. Some libraries are written by me and are potentially reusable. 
- **/experiments** - each subfolder stores the scripts and data for validation experiments.
- **/doc** - Electronics modules and components documentation from original manufactures.

All of the C++ projects are created with Visual Studio with Visual Micro plugin.

## Design Goals

The clamp firmware project aims to create a network of remote controlled actuators. A single PC with a USB attached radio (**SerialRadio**) is used to communicate with a network of distributed actuators (**ClampController**) within a indoor factory scenario over ISM band radio frequency with the following features:

- Master slave communication
- Acknowledgement of received messages
- Low latency
- Highly reliable

Each actuator (**ClampController**) is capable of controlling up to 2 DC motors to follow a trapezoidal motion profile (acceleration, constant speed, deceleration). 

- DC motors contains hall effect shaft encoder for feedback control
- The mechanical system needs to be homed. 
- The motor controller needs to detect and act upon motor stalling to avoid burning out the motor.
- The mechanical system needs operate at programmed speed.



## ClampController

List of Clamp Controller firmware documentations

- 01_SerialCommandController (TODO: Create Doc)
- [03_TokyoController (Detailed documentation)](clamp_controller/03_TokyoController.md)
- [04_CL3_Controller (Detailed documentation)](clamp_controller/04_CL3_Controller.md)

### Wireless Communication (General Description)

The CC1101 radio transceiver is used for the transport layer due to its sub-GHz frequency and good cost-effectiveness. This radio operates in half-duplex mode and does not have built-in flow control.

**Flow control:** For the required two-way communication and to avoid collision, *the master node always initiates communication* with a slave node. On the master node, this is implemented on application layer in python (refer to serial_radio_transport_driver in [clamp_controller repository](https://github.com/gramaziokohler/clamp_controller)). On the slave nodes, the ClampController is implemented to only respond to master-initiated communications.

**Reliability:** Acknowledgement-retransmission based protocol is used to create reliable communication over the unreliable physical layer (radio waves). This behavior is implemented on the application layer on the master node. Typically, a master issues a command to an addressed slave, the slave will acknowledge the message by sending a response back to the master. If the slave response is not received, the master needs to decide what happens, such as repeating a number of retry.

### Motor Control (General Description)

Bi-directional position control is achieved with a PID control loop ([libraries\PID](libraries\PID)) over the instantaneous positional target of a motion profile ([libraries\MotorController](libraries\MotorController)).

**Motion Profile:** Linear and 3-phase trapezoidal profile (accelerate, coast, decelerate) was [tested](experiments\motion_control_experiments\) and the results suggest a trapezoidal profile is sufficiently accurate.

**Encoder Choice:** Single-phase shaft encoder was used in early tests and was found to be inadequate for motion control. Laster designs all use two-phase shaft encoder.

## SerialRadio

[Details of USB Serial Radio Firmware (including message format)](serial_radio/Readme.md)

Credits
-------------

This repository was created by Pok Yin Victor Leung <leung@arch.ethz.ch> [@yck011522 ](https://github.com/yck011522) at [@gramaziokohler](https://github.com/gramaziokohler)

