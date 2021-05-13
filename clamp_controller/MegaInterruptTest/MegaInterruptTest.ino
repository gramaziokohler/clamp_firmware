/*
 Name:		06_SL3_Controller.ino
 Created:	2021-05-03
 Author:	leungp


 This controller continues the development from 03_TokyoController
 Controls 1 main motor (m1) and 2 auxiliary motors (m2, m3 for grippers)

 The controller can accept commands from USB Serial (\n termainated)
 Radio commands can also be accepted. (This controller's default address is '1')
 h              - Home the axis
 ?              - Print out status
 g[position]    - move to a given position (step), can be positive or negative value.
 i[0/1]			- Extend / Retract Gripper
 s              - immediately stop

 o[offset-pos]	- Set Homed Offset (step) (persistent) (NEW)
 v[velocity]    - Set Velocity in (step/s) (persistent)
 a[accel]		- Set Acceleration in (step/s^2) (persistent)
 e[error]		- Set Error-To-Stop (steps) (persistent)
 p[power]       - Set Maximum Power used in motor control. (range 0 to 100) (persistent) (NEW)
 x[1]			- Reset settings stored in EEPROM to default values.

 r[message]     - Send radio message to master (default address '0') e.g. rHello\n
 f[0/1]			- Enable Disable Radio Fix


 This controller is intended to use with "04_RadioPIDControllerV3" electronics module.
 The pin assignment is for SL3 Screwdriver Hardware (3 motors + 3 encoders + 2 homing switches for aux motors)

 The new feature in this controller include
- Gripper motor control and status feedback
- Designed for Ardhuino Mega with more IO pins
- DIP Switch now connectes to 4 pins instead of resistor ladder

*/

// Visual Micro "Optional Sketch Book" location should be set to the root path of the repo
// e.g. C:\Users\leungp\Documents\GitHub\clamp_firmware\

// The include statements are all in <angle brackets> because compiler need to look in the include folder for these files.
// For example "Double Quote" won't work for Encoder.h because it has folder structure in its nested imports.
// All libraries should ideally be located in libraries include folder.
// * clamp_firmware libraries are developed in this project. Located in \clamp_firmware\libraries

#include <EEPROM.h>         // Arduino default library for accessing EEPROM

#include <DCMotor.h>        // clamp_firmware library

#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>        // Encoder library from PJRC.COM, LLC - Paul Stoffregen http://www.pjrc.com/teensy/td_libs_Encoder.html
#include <MotorController.h>    //New Class in Development

#include <BufferedSerial.h>

#include <cc1101_nointerrupt.h>
#include <ccpacket.h>

//#include <Wire.h>


// #define SerialComment
//MOTOR_CONTROL_PRINTOUT (if defined) will print motor control error during motor movements.
// #define MOTOR_CONTROL_PRINTOUT

//SERIAL_MASTER_INIT_TALK_ONLY (if defined) will ensure the controller do not initiate any communication.
//	The controller will act as a slave node and only talk back when the master initiate a message.
//	This is crucial in a master managed time share network to avoid bus contention.
//	This feature can be disabled for debug use when the controller is connected directly to a PC.
//#define SERIAL_MASTER_INIT_TALK_ONLY

//Pins for Motor Driver M1 (Main Motor)
const uint8_t m1_driver_ena_pin = 46;             // the pin the motor driver ENA1 is attached to (PWM Pin)
const uint8_t m1_driver_in1_pin = 48;             // the pin the motor driver IN1 is attached to
const uint8_t m1_driver_in2_pin = 49;             // the pin the motor driver IN2 is attached to

//Pins for Motor Driver M2 (Gripper Motor)
const uint8_t m2_driver_ena_pin = 44;          // L298H ENABLEA Pin (Schematic: MB_ENA)
const uint8_t m2_driver_in1_pin = 40;          // L298H INPUT1 Pin (Schematic: MB_IN1)
const uint8_t m2_driver_in2_pin = 47;          // L298H INPUT2 Pin (Schematic: MB_IN2)

//Pins for Motor Driver M3 (Gripper Motor)
const uint8_t m3_driver_ena_pin = 45;          // L298H ENABLEB Pin (Schematic: MC_ENA)
const uint8_t m3_driver_in1_pin = 42;          // L298H INPUT3 Pin (Schematic: MC_IN1)
const uint8_t m3_driver_in2_pin = 43;          // L298H INPUT4 Pin (Schematic: MC_IN2)

//Pins for Motor encoder (Main Motor + 2 Gripper Motor)
const uint8_t m1_encoder1_pin = 2;             // Motor 1 encoder channel 1 (Schematic: MA_E1)
const uint8_t m1_encoder2_pin = 3;             // Motor 1 encoder channel 2 (Schematic: MA_E2)
const uint8_t m2_encoder1_pin = 19;            // Motor 2 encoder channel 1 (Schematic: MB_E1)
const uint8_t m2_encoder2_pin = 21;            // Motor 2 encoder channel 2 (Schematic: MB_E2)
const uint8_t m3_encoder1_pin = 20;            // Motor 3 encoder channel 1 (Schematic: MC_E1)
const uint8_t m3_encoder2_pin = 18;            // Motor 3 encoder channel 2 (Schematic: MC_E2)

//Pins for Homing Switch
const uint8_t m2_home_pin = 39;                 // Limit switch in INPUT_PULLUP Mode (Schematic: MB_SW)
const uint8_t m3_home_pin = 38;                 // Limit switch in INPUT_PULLUP Mode (Schematic: MC_SW)

//Pins for radio
const uint8_t radio_ss_pin = 53;                // Hardware SPI Interface
const uint8_t radio_mosi_pin = 51;              // Hardware SPI Interface
const uint8_t radio_miso_pin = 50;              // Hardware SPI Interface
const uint8_t radio_sck_pin = 52;               // Hardware SPI Interface
const uint8_t radio_gdo0_pin = A2;              // Input to sense incoming package

//Pins for Battery Monitor / DIP Switch / LED
const uint8_t battery_monitor_pin = A3;         // Analog Pin
const uint8_t dip_switch_pin_1 = 4;
const uint8_t dip_switch_pin_2 = 5;
const uint8_t dip_switch_pin_3 = 6;
const uint8_t dip_switch_pin_4 = 7;

const uint8_t grn_led_pin = 15;
const uint8_t blu_led_pin = 14;
const uint8_t org_led_pin = 13;

// ---- END OF PIN ASSIGNMENT  ----

//Tunings for motors
const double m1_kp = 0.005;                 // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile m1_kp = 0.040
const double m1_ki = 0.003;                 // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile m1_ki = 0.200
const double m1_kd = 0.0001;                // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile m1_kd = 0.0002

const double m2_kp = 0.005;                 // 
const double m2_ki = 0.003;                 // 
const double m2_kd = 0.0001;                // 

const double m3_kp = 0.005;                 // 
const double m3_ki = 0.003;                 // 
const double m3_kd = 0.0001;                // 

const double default_velocity = 500;			// Conservative speed
const double default_accel = 5000;               // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile m1_accel = 3000
const double default_error_to_stop = 200.0;         // Maximum absolute step error for the controller to stop itself without reaching the goal.
const long default_home_position_step = 0;
const double default_power = 1.0;			// Default to full power
const int motor_run_interval = 10;          // Motor PID sample Interval in millis()

// Settings for radio communications
const char radio_master_address = '0';      // Address of default master radio
char radio_selfAddress = '1';       // Address default to char '1'
const CFREQ frequency = CFREQ_433;          // CFREQ_868 CFREQ_915 CFREQ_433 CFREQ_918
const byte channelNumber = 0;
const uint8_t radio_power = PA_LongDistance;    // PA_MinimalPower PA_ReducedPower PA_LowPower PA_LongDistance
constexpr byte radio_syncWord[2] = { 01, 27 };


// ---- END OF MODIFIABLE SETTINGS - Do not modify below ----

// Initialize motion control objects
DCMotor Motor1(m1_driver_ena_pin, m1_driver_in1_pin, m1_driver_in2_pin);
Encoder Encoder1(m1_encoder1_pin, m1_encoder2_pin);
MotorController MotorController1(&Motor1, &Encoder1, m1_kp, m1_ki, m1_kd, default_accel, motor_run_interval, default_error_to_stop, true, false);

DCMotor Motor2(m2_driver_ena_pin, m2_driver_in1_pin, m2_driver_in2_pin);
Encoder Encoder2(m2_encoder1_pin, m2_encoder2_pin);
MotorController MotorController2(&Motor2, &Encoder2, m2_kp, m2_ki, m2_kd, default_accel, motor_run_interval, default_error_to_stop, true, false);

DCMotor Motor3(m3_driver_ena_pin, m3_driver_in1_pin, m3_driver_in2_pin);
Encoder Encoder3(m3_encoder1_pin, m3_encoder2_pin);
MotorController MotorController3(&Motor3, &Encoder3, m3_kp, m3_ki, m3_kd, default_accel, motor_run_interval, default_error_to_stop, true, false);

// Variables for serial communication
byte incomingByte;
char status_string[61];

// Variables for battery monitor
int batt_value;

// Variables for communications
BufferedSerial bufferedSerial(1);
CC1101 radio;
uint8_t radio_partnum, radio_version, radio_marcstate;	// Stores the register values of the radio fetched at startup.
unsigned long radio_last_receive_millis = 0;
unsigned long radio_unfrozen_applied_millis = 0;
boolean radio_fix_enabled = true;
boolean serial_printout_enabled = false;

// Variables for profiling
unsigned long profile_start_micros = 0;
unsigned long profile_end_micros = 0;


void setup() {

	// Initialize Serial
	bufferedSerial.serialInit();
}

void loop() {

	MotorController2.moveToPosition(5000, 500);
	MotorController3.moveToPosition(5000, 500);
	while (MotorController2.isMotorRunning()) {

		if (MotorController2.run()) {
			Serial.print(MotorController2.currentPosition());
			Serial.print(',');
			Serial.println(MotorController3.currentPosition());
		}
		MotorController3.run();
	}
	delay(500);

	MotorController2.moveToPosition(0, 1000);
	MotorController3.moveToPosition(0, 1000);
	while (MotorController2.isMotorRunning()) {
		if (MotorController2.run()) {
			Serial.print(MotorController2.currentPosition());
			Serial.print(',');
			Serial.println(MotorController3.currentPosition());
		}
		MotorController3.run();
	}
	delay(500);
}




// Status Reporting
char* get_current_status_string() {
	unsigned int i = 0; // This variable keep count of the output string.
	i += snprintf(status_string + i, 60 - i, "%u,%ld,%ld,%d,%i,%ld,%ld,%ld,%ld",
		get_status_code(),
		(long)MotorController1.currentPosition(),
		(long)MotorController1.currentTarget(),
		(int)(MotorController1.currentMotorPowerPercentage() * 100.0),
		batt_value,
		(long)MotorController2.currentPosition(),
		(long)MotorController2.currentTarget(),
		(long)MotorController3.currentPosition(),
		(long)MotorController3.currentTarget()
	);
	return status_string;
}

byte get_status_code() {
	byte code = 0;
	bitWrite(code, 0, MotorController1.isHomed());
	bitWrite(code, 1, (MotorController1.isMotorRunning()));
	bitWrite(code, 2, (MotorController1.isDirectionExtend()));
	return code;
}


void run_status_light() {
	// Status evaluation is continaed within this function.
	// This functions's checking peirod is equal to the blicking frequency of the LED

	const unsigned long STATUS_LIGHT_PEIROD_MILLIS = 200;
	static unsigned long next_run_time = 0;
	static boolean blinking_high_low = true; // The value of this boolean is flipped every time status is checked

	if (millis() > next_run_time) {
		next_run_time = millis() + STATUS_LIGHT_PEIROD_MILLIS;

		// Toggle blinking on/off value
		blinking_high_low = !blinking_high_low;

		// ** Green Status Light **
		//- Long Lit = Power On
		//- Blink = Low Battery (0% estimate 880)
		const float LOW_BATT_THRESHOLD = 880;
		if (batt_value < LOW_BATT_THRESHOLD) {
			digitalWrite(grn_led_pin, blinking_high_low);
		}
		else {
			digitalWrite(grn_led_pin, HIGH);
		}

		// ** Orange Status Light **
		//- Long Lit = Motor Moving
		//- Blink = Communication not received for over 1 sec.
		//- Off = Standby / Normal
		const unsigned long RADIO_NO_RECEIVE_THRESHOLD = 1000;
		if (MotorController1.isMotorRunning()) {
			digitalWrite(org_led_pin, HIGH);
		}
		else if (millis() - radio_last_receive_millis >= RADIO_NO_RECEIVE_THRESHOLD) {
			digitalWrite(org_led_pin, blinking_high_low);
		}
		else {
			digitalWrite(org_led_pin, LOW);
		}
	}
}
