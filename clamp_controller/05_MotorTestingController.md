# Motor Testing Controller

This is a motion profile based controller for testing motors and drive train under load. The primary purpose is to execute motion similar to the clamp controllers and monitor the position, PID output (PWM value), error from motion profile during the execution of a motion.

These information can be collected by a Serial monitor for analysis. Since PID settings can be modified via Serial, this controller can also be used for  automated testing for PID tuning.

This controller is largely a simplified down version of 04 CL3 controller. It does not have radio capability in favor of faster PID refresh rate and serial report during motion.



Only channel one (OUT1 OUT2is used

## Wiring

The wiring necessary between the Arduino and motor driver (XY-160D) is similar to [CL3 Controller](./04_CL3_Controller.md). 

- Only channel one (OUT1 OUT2) is used. 

- No homing switch is expected.
- Battery voltage sensing is available as the device is expected to operate in battery mode.  (maximum 16.8V)

**Driver input voltage:** DC 6.5V - 27V
**Rated output current:**  7A (Per Channel)
**PWM frequency range** 0-10 kHz 



| Connection        | Arduino Pin | Name in Code (One Motor) |
| ----------------- | ----------- | ------------------------ |
| USB               | D0          |                          |
| USB               | D1          |                          |
| Motor Encoder     | D2          | m1_encoder1_pin          |
| Motor Encoder     | D3          | m1_encoder2_pin          |
| Motor Driver      | D4          | m1_driver_in1_pin        |
| Motor Driver      | D5          | m1_driver_ena_pin        |
| Motor Driver      | D6          |                          |
| Motor Driver      | D7          | m1_driver_in2_pin        |
| Motor Driver      | D8          |                          |
| Motor Driver      | D9          |                          |
| ~~Radio~~         | D10         |                          |
| ~~Radio~~         | D11         |                          |
| ~~Radio~~         | D12         |                          |
| ~~Radio~~         | D13         |                          |
| ~~Radio~~         | A0          |                          |
| ~~Homing Switch~~ | A1          | m1_home_pin              |
| ~~Homing Switch~~ | A2          |                          |
| ~~LED~~           | A3          |                          |
| ~~Motor Encoder~~ | A4          |                          |
| ~~Motor Encoder~~ | A5          |                          |
| ~~DIP Switch~~    | A6          |                          |
| Battery Sense     | A7          | battery_monitor_pin      |

## Communication

### Serial Commands

| Command                             | Format            | Notes                                                        | Default | Example    |
| ----------------------------------- | ----------------- | ------------------------------------------------------------ | ------- | ---------- |
| Goto                                | g[position]`\n`   | [position] can be any signed long integer<br />Value counted in step |         | g1000`\n`  |
| Stop                                | s`\n`             |                                                              |         | s`\n`      |
| Home                                | h`\n`             | Home commands functions as to reset position to 0            |         | h`\n`      |
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

### Position

The controller works in step position related to the encoder step.

|            | Encoder steps per rev | Gearbox Multiply | steps / rev | Linear Screw Pitch | steps / mm |
| ---------- | --------------------- | ---------------- | ----------- | ------------------ | ---------- |
| 42GP-775   | 30                    | ~ 1:49           | 1470        |                    |            |
| 36GP-555   | ?                     | ?                |             | ?                  |            |
| GW4058-555 | 34                    | ~ 1:108          | 3672        | 4                  | 918        |

The motor has **3672 steps per rev.** / with the 1204 lead scew: **918 steps per mm**

Homing Switch Position have an approximately 93mm jaw opening. After homing the controller reset to **step -1650** (~-1.8mm). Typical use should then go towards zero (+ve direction) by issuing `g0\n` command.



### Firmware upload and commission

1. Connect to Serial Port (USB),
2. Upload firmware
3. Confirm Radio address is correct
4. `x1 `Reset Settings
5. Press Reset Button to make settings effective
6. Attach Battery
7. `p50` to set power level
8. `g100` to confirm Encoder / DC motor wiring direction (Flip if necessary)
9. `H` and trigger switch manually to confirm switch is functional.
10. `H` and allow automatic homing
11. Measure jaw opening and calculate offset by [opening * 918]
12. `o[offset value]` j



