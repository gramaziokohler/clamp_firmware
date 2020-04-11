# clamp_firmware
Firmware for embedded Arduino on remote-controlled actuators (clamps).

This repo is part of the [Robotic Assembled Timber Structures with Integral Timber Joints](https://github.com/gramaziokohler/integral_timber_joints) project. 

## Repo folder structure

- **/clamp_controller** - Contains one Visual Studio solutions that contains all clamp controller projects. Sub folder contains different VC++ projects for various controller.
- **/serial_radio** - Contains one Visual Studio solution for the Serial Radio. Sub folder contains different VC++ projects for various controller.
- **/libraries** - stores all the internal and external libraries for the c++ compiler. Some libraries are written by me and are potentially reusable. 
- **/experiments** - each subfolder stores the scripts and data for validation experiments.
- **/doc** - Electronics modules and components documentation from original manufactures.

All of the C++ projects are created with Visual Studio with Visual Micro plugin.

## Design Goals

The clamp firmware project aims to create a network of remote controlled actuators. A single PC with a USB attached radio (**SerialRadio**) can be used to communicate with a network of distributed actuators (**ClampController**) within a indoor factory floor over ISM frequency. 

- Master slave communication
- Acknowledgement protocol
- Low latency
- Highly reliable

Each actuator (ClampController) is capable of controlling up to 2 DC motors to follow a trapezoidal motion profile (acceleration, constant speed, deceleration). 

- DC motors contains hall effect shaft encoder for feedback control
- The mechanical system needs to be homed. 
- The motor controller needs to detect and act upon motor stalling to avoid burning out the motor.
- The mechanical system needs operate at programmed speed.

## Wireless Communication

The CC1101 radio transceiver is used for the transport layer due to its sub-GHz frequency and good cost-effectiveness. This radio operates in half-duplex mode and does not have built-in flow control.

For the required two-way communication and to avoid collision, **the master node always initiates communication** with a slave node. On the master node, this is implemented on application layer in python [Clamp Commander Project]. On the slave nodes, the **ClampController** is implemented to only respond to master-initiated communications.

An acknowledgement-retransmission based protocol is used to create reliable communication over the unreliable physical layer (radio waves). This behavior is implemented on the application layer on the master node. The typical behavior is where a master issues a command to an addressed slave, the slave will acknowledge the message by sending a response. The slave response is not acknowledged by the master. The master node needs to decide what happens when the slave do not acknowledge.

## Motor Control



## Firmware structure

Firmware for Clamp Controllers

- Clamp Controller 

Firmware for USB Serial Radio

- 

Credits
-------------

This repository was created by Pok Yin Victor Leung <leung@arch.ethz.ch> [@yck011522 ](https://github.com/yck011522) at [@gramaziokohler](https://github.com/gramaziokohler)

