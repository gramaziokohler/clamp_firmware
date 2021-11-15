# SL1 Clamp Controller

## Wiring

### JZ-3615 motor driving board

**Driver input voltage:** DC 6V - 36V
**Number of Channels:** 1
**Rated output current:**  15A (Per Channel)
**PWM frequency range** 0-20 kHz (PWM signal at ENA input is used to regulate speed)

| Pin Function                  | Label       | Connection         | Cable Color | Arduino Pin | Name in Code      |
| ----------------------------- | ----------- | ------------------ | ----------- | ----------- | ----------------- |
| Drive Power Input +ve         | 3-36V       | Battery Positive   | Red         |             |                   |
| Drive Power Input -ve         | PGND        | Battery Negative   | Blk         |             |                   |
| Power Output +ve              | P3          | M1 +ve             | Red         |             |                   |
| Power Output -ve              | P3 (OUTPUT) | M1 -ve             | Blk         |             |                   |
| Digital Power +ve             | 5V0         | Arduino 5V         |             | 5V          |                   |
| Digital Input - Speed         | PWM         | Arduino GPIO (PWM) |             | 46          | m1_driver_ena_pin |
| Digital Input - Channel 1 - 1 | IN1         | Arduino GPIO       |             | 48          | m1_driver_in1_pin |
| Digital Input - Channel 1 - 2 | IN2         | Arduino GPIO       |             | 49          | m1_driver_in2_pin |
| Digital Power -ve             | COM         | Arduino Ground     |             | GND         |                   |

### HJX50RNA27i-X8001BM Planetary Gearbox 1:22.21 + NMRV 1:20 Worm Gearbox

The integrated hall sensor on the DC motors have 16 steps per rev per channel. Effectively **64 steps per rev**. (100kHz max). After the two gear boxes, the conversion is **28428 steps per rev.**

The pull rod has 5mm pitch, translating to **5685.6 steps per mm**

| Pin Function    | Label | Connection  | Cable Color | Arduino Pin | Name in Code    |
| --------------- | ----- | ----------- | ----------- | ----------- | --------------- |
| Encoder         | C1    | Arduino     | Yel         | 2           | m1_encoder1_pin |
| Encoder         | C2    | Arduino     | Grn         | 3           | m1_encoder2_pin |
| Encoder Power + | VCC   | Arduino     | Blu         | 5V          | -               |
| Encoder Power - | GND   | Arduino     | Blk         | Gnd         | -               |
| Motor Power     | M1    | Driver OUT1 | Red         |             | -               |
| Motor Power     | M2    | Driver OUT2 | Wht         |             | -               |

### CHP36GP-3429 Planetary Gearbox 1:99.5075

Three stage gear box with ratio (determined by the number of teeth on Ring vs Sun: (Teeth-on-ring) / (Teeth-on-sun) + 1) = (46/17+1) x (46/11+1) x (46/11+1)

The integrated hall sensor on the DC motors have 11 steps per rev per channel. Effectively **44 steps per rev**. (100kHz max). After the two gear boxes, the conversion is **4378 steps per rev.**

The Gripper Pin Mechanism has about 47mm travel on M12 with 1.75mm Pitch. Total travel is **117589 steps**

Motor 2 (PCB: Motor B)

| Pin Function    | Label (Motor) | Label PCB | Cable Color | Arduino Pin   | Name in Code    |
| --------------- | ------------- | --------- | ----------- | ------------- | --------------- |
| Motor Power     | M1            | MB+       |             | (L298HN OUT1) |                 |
| Motor Power     | M2            | MB-       |             | (L298HN OUT2) |                 |
| Encoder         | C1            | MB_En1    |             | 19            | m2_encoder1_pin |
| Encoder         | C2            | MB_En2    |             | 21            | m2_encoder2_pin |
| Encoder Power + | VCC           | +5V       |             | 5V            | -               |
| Encoder Power - | GND           | GND       |             | Gnd           | -               |

Motor 3 (PCB: Motor C)

| Pin Function    | Label (Motor) | Label PCB | Cable Color | Arduino Pin   | Name in Code    |
| --------------- | ------------- | --------- | ----------- | ------------- | --------------- |
| Motor Power     | M1            | MC+       |             | (L298HN OUT3) |                 |
| Motor Power     | M2            | MC-       |             | (L298HN OUT4) |                 |
| Encoder         | C1            | MC_En1    |             | 20            | m3_encoder1_pin |
| Encoder         | C2            | MC_En2    |             | 18            | m3_encoder2_pin |
| Encoder Power + | VCC           | +5V       |             | 5V            | -               |
| Encoder Power - | GND           | GND       |             | Gnd           | -               |

### Homing switch for gripper motors

Utilizing Arduino's Internal pull up resistor (47k). 100nF capacitor is added for noise filtering between NC and GND, close to the Arduino.

**Normal closed** behavior used. (Broken wire can be detected) Triggered state (pressed) is electrical **HIGH**.

| Pin Function  | Label | Connection | Cable Color | Arduino Pin | Name in Code |
| ------------- | ----- | ---------- | ----------- | ----------- | ------------ |
| Switch1 - COM | 1     | Arduino    | Black       | GND         | -            |
| Switch1 - NC  | 2     | Arduino    | Red         | 39          | m2_home_pin  |
| Switch1 - NO  | 3     | ~~n/c~~    | -           | -           | -            |
| Switch2 - COM | 1     | Arduino    | Black       | GND         | -            |
| Switch2 - NC  | 2     | Arduino    | Red         | 38          | m3_home_pin  |
| Switch2 - NO  | 3     | ~~n/c~~    | -           | -           | -            |

### Battery Sense and Regulation

| Pin Function | Label | Connection | Cable Color | Arduino Pin | Name in Code        |
| ------------ | ----- | ---------- | ----------- | ----------- | ------------------- |
| COM          |       | Arduino    | Blk         | GND         | -                   |
| Input        |       | Battery +  | Red Stripe  |             | -                   |
| Output       |       | Arduino    | Red Stripe  | A7          | battery_monitor_pin |

Battery sense is a simple Voltage Divider

R1 = 47kOhm

R2 = 20kOhm

Theoretical Voltage at 16.8V = 5.01V (1024/1024)

Theoretical Voltage at 14.4V = 4.30V (880/1024)

Theoretical Resolution from 0 to 100% = 144 steps 

According to some questionable source:

4.20v = 100%
4.03v = 76%
3.86v = 52%
3.83v = 42%
3.79v = 30%
3.70v = 11%
3.6?v = 0%

### CC1101 Radio

| Pin Function              | Label | Connection | Cable Color | Arduino Pin | Name in Code   |
| ------------------------- | ----- | ---------- | ----------- | ----------- | -------------- |
| 5V Power +                | VCC   | Arduino    |             | 5V*         |                |
| Power Ground              | GND   | Arduino    |             | GND         |                |
| Slave Select (SS)         | CSN   | Arduino    |             | 10          | radio_ss_pin   |
| Master Output Slave Input | SI    | Arduino    |             | 11          | radio_mosi_pin |
| Master Input Slave Output | SO    | Arduino    |             | 12          | radio_miso_pin |
| Serial Clock              | SCK   | Arduino    |             | 13          | radio_sck_pin  |
| General Output 0          | GO0   | Arduino    |             | A0          | radio_gdo0_pin |
| General Output 2          | GO2   | ~~n/c~~    | ~~n/c~~     | ~~n/c~~     | ~~n/c~~        |

Note: Nano Hardware SPI Pins: SPI: 10 (SS), 11 (MOSI), 12 (MISO), 13 (SCK). These should be followed.

Note: Despite CC1101 requires VCC = 3.3V, the TELESKY modules I got cannot operate in 3.3V, it needs 5.0V. I do not have a spec sheet or the schematic, I suspect it has an onboard Voltage converter.

### Status Light

**Green Status Light**

- Long Lit = Power On 
- Blink = Low Battery (Dangerously Low) 

Note: Low battery is estimated roughly at 3.6V x 4  = 14.4V (LOW_BATT_THRESHOLD = 880) because the controller do not carry the battery sense calibration value.

**Blue Status Light (Communication)**

- Long Lit = Motor Moving (Override the Blink Status)
- Blink = Communication not received for over 1 sec.
- Off = Standby / Normal 

**Orange Status Light**

- Long Lit = Main Motor (1) or Gripper Motor (2/3) Moving
- Blink = Gripper Motor Stuck (TODO)
- Off = Standby / Normal

### **DIP Switch**

A DIP switch with a resistor ladder. Occupy only a sinlge analog pin. SW1 is the Most Significant Bit

This can be used for changing settings easily without recompilation. Such as Radio address. 

| R0   | R1   | R2   | R3   | R4   |
| ---- | ---- | ---- | ---- | ---- |
| 1.6k | 2.2K | 4.7k | 10k  | 20k  |



| Arduino Pin | Name in Code   |
| ----------- | -------------- |
| A6          | dip_switch_pin |

**Radio Address Setting**

Tested with battery attached, 5V is provided 7805 regulator.

| SW1  | SW2  | SW3  | SW4  | Analog Reading <br />(Board 1) | Analog Reading <br />(Board 2) | Address<br />(char) |
| ---- | ---- | ---- | ---- | ------------------------------ | ------------------------------ | ------------------- |
| 0    | 0    | 0    | 0    | 0                              | 0                              | 1                   |
| 0    | 0    | 0    | 1    | 73                             | 74                             | 2                   |
| 0    | 0    | 1    | 0    | 140                            | 141                            | 3                   |
| 0    | 0    | 1    | 1    | 198                            | 197                            | 4                   |
| 0    | 1    | 0    | 0    | 258                            | 257                            | 5                   |
| 0    | 1    | 0    | 1    | 302                            | 301                            | 6                   |
| 0    | 1    | 1    | 0    | 341                            | 340                            | 7                   |
| 0    | 1    | 1    | 1    | 376                            | 375                            | 8                   |
| 1    | 0    | 0    | 0    | 431                            | 430                            | 9                   |
| 1    | 0    | 0    | 1    | 457                            | 457                            | 10                  |
| 1    | 0    | 1    | 0    | 482                            | 481                            | 11                  |
| 1    | 0    | 1    | 1    | 504                            | 503                            | 12                  |
| 1    | 1    | 0    | 0    | 529                            | 528                            | 13                  |
| 1    | 1    | 0    | 1    | 547                            | 546                            | 14                  |
| 1    | 1    | 1    | 0    | 564                            | 564                            | 15                  |
| 1    | 1    | 1    | 1    | 580                            | 580                            | 16                  |





### Arduino Nano Wiring Overview

![https://i.stack.imgur.com/W9Ril.png](https://i.stack.imgur.com/W9Ril.png)



| Connection    | Arduino Pin | Name in Code (One Motor) | Name in Code (Two Motor) |
| ------------- | ----------- | ------------------------ | ------------------------ |
| USB           | D0          |                          |                          |
| USB           | D1          |                          |                          |
| Motor Encoder | D2          | m1_encoder1_pin          | m1_encoder1_pin          |
| Motor Encoder | D3          | m1_encoder2_pin          | **m2_encoder1_pin**      |
| Motor Driver  | D4          | m1_driver_in1_pin        | m1_driver_in1_pin        |
| Motor Driver  | D5          | m1_driver_ena_pin        | m1_driver_ena_pin        |
| Motor Driver  | D6          |                          | m2_driver_ena_pin        |
| Motor Driver  | D7          | m1_driver_in2_pin        | m1_driver_in2_pin        |
| Motor Driver  | D8          |                          | m2_driver_in1_pin        |
| Motor Driver  | D9          |                          | m2_driver_in2_pin        |
| Radio         | D10         | radio_ss_pin             | radio_ss_pin             |
| Radio         | D11         | radio_mosi_pin           | radio_mosi_pin           |
| Radio         | D12         | radio_miso_pin           | radio_miso_pin           |
| Radio         | D13         | radio_sck_pin            | radio_sck_pin            |
| Radio         | A0          | radio_gdo0_pin           | radio_gdo0_pin           |
| Homing Switch | A1          | m1_home_pin              | m1_home_pin              |
| Homing Switch | A2          |                          | **m2_home_pin**          |
| LED           | A3          | status_led_pin           | status_led_pin           |
| Motor Encoder | A4          |                          | **m1_encoder2_pin**      |
| Motor Encoder | A5          |                          | **m2_encoder2_pin**      |
| DIP Switch    | A6          | dip_switch_pin           | dip_switch_pin           |
| Battery Sense | A7          | battery_monitor_pin      | battery_monitor_pin      |

## Communication

### Serial Commands

| Command                             | Format            | Notes                                                        | Default | Example    |
| ----------------------------------- | ----------------- | ------------------------------------------------------------ | ------- | ---------- |
| Goto                                | g[position]`\n`   | [position] can be any signed long integer<br />Value counted in step |         | g1000`\n`  |
| Stop                                | s`\n`             |                                                              |         | s`\n`      |
| Home                                | h`\n`             |                                                              |         | h`\n`      |
| Set Velocity (persistent)           | v[velocity]`\n`   | [velocity] can be any signed double<br />Value counted in step/s | 500     | v2000`\n`  |
| Set Acceleration (persistent)       | a[accel]`\n`      | [accel] can be any signed double<br />Value counted in step/s^2 | 5000    | a5000`\n`  |
| Set error_to_stop (persistent)      | e[error]`\n`      | [error] can be any signed double<br />Value counted in step  | 200     | e300`\n`   |
| Set home_position_step (persistent) | o[offset-pos]`\n` | [offset-pos] can be any signed long<br />Value counted in step | 0       | p93500`\n` |
| Set Maximum Power (persistent)      | p[max_power]`\n`  | [max_power] can be any float between 0.0 to 100.0<br />Value is percentage of maximum power output. | 100     | p80`\n`    |
| Get Status Message                  | ?`\n`             | See table below                                              |         | ?`\n`      |

All commands are non-blocking. 

A newly arrived command will override an older command. 

- **Goto** command will go to new target even if previous goto command is not completed.
- **Stop** command can stop **Goto** or **Home** motions at anytime.
- **Set Velocity** command will not affect the speed of ongoing motion. It will affect the next Goto motion.
- **Set Maximum Power** have immediate effect on ongoing motion. 
- **Get Status Message** command prints the status string that includes status code, current position, current target, current power and battery value. See **Status Message** section below.

### Radio Communication

Radio communication is performed with a USB to Radio dongle. This dongle runs Serial2Radio_Tokyo.ino sketch. Which preconfigures frequency / channel / sync word etc for the communication. **Do not** connect LCD screen to dongle.

Commands similar to the Serial commands above are sent to the dongle with the addition of two header bytes in front for addressing. The radio is configured to use '\n' as End-Of-Message Termination character, similar to the Serial Command, therefore one '\n' is sufficient

**Format**: Clamp Address + Master Address = '0' + Serial Command 

**Example**: `10h\n` means sending to clamp '1' from master '0' the home command  

When any message is received at the clamp controller, it will reply with a full status message regardless of what commands are given. If the message is an instruction, it will first execute the instruction before replying the status message. Goto instruction is non-blocking, the controller will set the target position and then reply the status message.

**Note**: The **Get Status Message**   `10?\n` command should not be used to get a remote response of the status message because it causes both the **Arduino Serial port to perform a printout** and **the radio to transmit the same status**. This unnecessarily slows down the controller. An empty message with only address header, such as`10\n` should be used to get back the status via radio. 

### Status Message

| Value Item                    | Meaning                                                      | Type / Range      |
| ----------------------------- | ------------------------------------------------------------ | ----------------- |
| status_code                   | Bit [0] = Homed<br />Bit [1] = MotorRunning<br />Bit [2] = DirectionExtending | (byte) 0 -7       |
| m1currentPosition             | PID Loop current position                                    | (long int)        |
| m1currentTarget               | Current PID positional control target.                       | (long int)        |
| m1currentMotorPowerPercentage | Current PID output for motor driver                          | (int) -100 to 100 |
| battValue                     | Raw Batt Voltage Value                                       | (int) 0 - 1024    |
| m2currentPosition             | Gripper Motor current Position                               | (long int)        |
| m2currentTarget               | Gripper Motor current Target                                 | (long int)        |
| gripper_state                 | 0 = GRIPPER_NotHomed<br />1 = GRIPPER_Extending<br />2 = GRIPPER_Retracting<br />3 = GRIPPER_Extended<br />4 = GRIPPER_Retracted<br />5 = GRIPPER_ExtendFail<br />6 = GRIPPER_RetractFail | (int) 0-6         |

## Operational Notes

### Setting Device Address

| SW1  | SW2  | SW3  | Device Address (char) | Device Address (int) |
| ---- | ---- | ---- | --------------------- | -------------------- |
| 0    | 0    | 0    | 1                     | 49                   |
| 0    | 0    | 1    | 2                     | 50                   |
| 0    | 1    | 0    | 3                     | 51                   |
| 0    | 1    | 1    | 4                     | 52                   |
| 1    | 0    | 0    | 5                     | 53                   |
| 1    | 0    | 1    | 6                     | 54                   |
| 1    | 1    | 0    | 7                     | 55                   |
| 1    | 1    | 1    | 8                     | 56                   |

**During operation:** Clamp 1 address is '1'. Clamp 2 address is '2'.

Simple.

### Digital Twin Declaration

SerialCommanderTokyo.py implemented the two following lines to initiate the ClampModel

```
self.clamp1 = ClampModel('1', 918, 95.0, 94.0, 225.0, 860.0, 1004.0)

self.clamp2 = ClampModel('2', 918, 95.0, 94.0, 225.0, 860.0, 1004.0)
```



### Position

Clamp Jaw **Extension** Direction is **Positive**

The motor has **3672 steps per rev.** / with the 1204 lead scew: **918 steps per mm**

Homing Switch Position have an approximately 93mm jaw opening. After homing the controller reset to **step -1650** (~-1.8mm). Typical use should then go towards zero (+ve direction) by issuing `g0\n` command.

| Description                    | Controller Position (step) | Controller Position (mm) | Jaw Spacing (mm) |
| ------------------------------ | -------------------------- | ------------------------ | ---------------- |
| Homed Position                 | -1650                      | -1.8                     | 93.2             |
| Zero / <br />Closed Jaw (Thin) | 0                          | 0                        | 95               |
| Closed Jaw (Thick)             | 13770                      | 15                       | 110              |
| Jaw Engage Joint               | 64260                      | 70                       | 165              |
| Open Jaw                       | 114750                     | 125                      | 220              |
| Maximum Extension              | 119340                     | 130                      | 225              |

### Speed

Speed from 1mm/s to 5mm/s had been tested to be stable and produce good torque.

### firmware upload and commission

1. Connect to Serial Port (USB),
2. Upload firmware
3. Confirm Radio address is correct
4. `x1 `Reset Settings
5. Press Reset Button to make settings effective
6. Attach Battery
7. `P50` to set power level
8. `G100` to confirm Encoder / DC motor wiring direction (Flip if necessary)
9. `H` and trigger switch manually to confirm switch is functional.
10. `H` and allow automatic homing
11. Measure jaw opening and calculate offset by [opening * 918]
12. `o[offset value]` j



## Implemented Clamps

| Address | Hardware | Encoder D2 | Encoder D3 | Out 1 | Out 2 |
| ------- | -------- | ---------- | ---------- | ----- | ----- |
| 1       | CL3      | Green      | Yellow     | Red   | White |
| 2       | CL3      | Yellow     | Green      | Red   | White |
| 3       | CL3M     | Yellow     | Green      | Red   | White |
| 4       | CL3M     |            |            |       |       |

| Address | Homed Position - Jaw Width (mm) | Offset x918 (steps) |
| ------- | ------------------------------- | ------------------- |
| 1       | 103.2                           | 94737               |
| 2       | 101.9                           | 93544               |
| 3       | 100.5                           | 92259               |
| 4       | 100.7                           | 92442               |



### Power Level / Force

A quick test is performed on Clamp #1 with a power supply set at 14.4V (max.10A). This test try to relate the power settings to the amount of output force:

| Power Percentage | Approximate Force (N) |
| ---------------- | --------------------- |
| 20               | 450                   |
| 30               | 750                   |
| 40               | 1140                  |
| 50               | 1600                  |
| 60               | 2030                  |

Test was stopped after 60% due to the fear of destroying the hardware. Deflection of the clamp component is observed at 60%.