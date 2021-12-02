# Motor Testing Controller

This is a motion profile based controller for testing motors and drive train under load. The primary purpose is to execute motion similar to the clamp controllers and monitor the position, PID output (PWM value), error from motion profile during the execution of a motion.

These information can be collected by a Serial monitor for analysis. Since PID settings can be modified via Serial, this controller can also be used for  automated testing for PID tuning.

This controller is largely a simplified down version of 04 CL3 controller. It does not have radio capability in favor of faster PID refresh rate and serial report during motion.

This motor controller is a continuation from 03_TokyoController but with PID settings accessible using serial port. The accompanying Octave file can be used to control the motor trail.



Sketch can be compiled with Arduino IDE by including the libraries in the ./libraries folder in build path.

# Usage

After uploading perform EEPROM reset with `x1`. Hardware reset.

The returned result format for `?` is in format of `">%u,%ld,%ld,%ld,%d,%i"` 

- Status Code
- `last_movement_start_time`
- `currentPosition`
- `currentTarget`
- `currentMotorPowerPercentage x 100`
- `batt_value`

Open `Test_Basic.m` in Octave, 

- select the correct COM port

- Adjust the movement command `srl_write(s1, "g5920\n") ;` if necessary. Positive is screwing tightening direction 
- Run and observe the output `result_*.csv` file and the accompany image of the plotted Position / Power / Error graph.

Determine fixed test parameters

- Determine typical running speed in steps per sec.
- Determine a reasonable acceleration and set setting such as `a2000`
- Make sure test distance (movement command) has acceleration, constant speed and deceleration phase.

Run test with different PID values.
