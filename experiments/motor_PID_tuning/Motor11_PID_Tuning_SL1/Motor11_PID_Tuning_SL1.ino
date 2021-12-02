/*
 Name:		04_CL3_Controller.ino
 Created:	2020-08-09
 Author:	leungp


 This controller continues the development from 03_TokyoController
 Persistent settings are added to reduce need of changing this code for different hardware config.
 The controller is finalized to include usability features and attempts to isolate functions into reusable classes.

 The controller can accept commands from USB Serial (\n termainated)
 Radio commands can also be accepted. (This controller's default address is '1')
 h              - Home the axis
 ?              - Print out status
 g[position]    - move to a given position (step), can be positive or negative value.
 s              - immediately stop

 o[offset-pos]	- Set Homed Offset (step) (persistent) (NEW)
 v[velocity]    - Set Velocity in (step/s) (persistent)
 a[accel]		- Set Acceleration in (step/s^2) (persistent)
 e[error]		- Set Error-To-Stop (steps) (persistent)
 p[power]       - Set Maximum Power used in motor control. (range 0 to 100) (persistent) (NEW)
 x[1]			- Reset settings stored in EEPROM to default values.

 The controller now uses a PID controller for the motors for position control
 to follow a motion profile.

 This controller is intended to use with "03_RadioPIDControllerV2" electronics module.
 The pin assignment is for CL3 Clamp Hardware (one encoder + motor + homing switch)

 The new feature in this controller include
 - Configurable max-motor-power.
 - Persistent settings.
 - LED status light output.
*/

// Visual Micro "Optional Sketch Book" location should be set to the root path of the repo
// e.g. C:\Users\leungp\Documents\GitHub\clamp_firmware\

// The include statements are all in <angle brackets> because compiler need to look in the include folder for these files.
// For example "Double Quote" won't work for Encoder.h because it has folder structure in its nested imports.
// All libraries should ideally be located in libraries include folder.
// * clamp_firmware libraries are developed in this project. Located in \clamp_firmware\libraries

#include <EEPROM.h>         // Arduino default library for accessing EEPROM

//#include "../../libraries/MotorController/DCMotor.h"
//#include "../../libraries/MotorController/MotorController.h"
//#include "../../libraries/Encoder/Encoder.h"
//#include "../../libraries/BufferedSerial/BufferedSerial.h"

#include <DCMotor.h>        // clamp_firmware library
#include <Encoder.h>        // Encoder library from PJRC.COM, LLC - Paul Stoffregen http://www.pjrc.com/teensy/td_libs_Encoder.html
#include <MotorController.h>    //New Class in Development
#include <BufferedSerial.h>

// #define SerialComment
//MOTOR_CONTROL_PRINTOUT (if defined) will print motor control error during motor movements.
// #define MOTOR_CONTROL_PRINTOUT

//SERIAL_MASTER_INIT_TALK_ONLY (if defined) will ensure the controller do not initiate any communication.
//	The controller will act as a slave node and only talk back when the master initiate a message.
//	This is crucial in a master managed time share network to avoid bus contention.
//	This feature can be disabled for debug use when the controller is connected directly to a PC.
//#define SERIAL_MASTER_INIT_TALK_ONLY


//Pins for Motor Driver M1
const uint8_t m1_driver_ena_pin = 46;             // the pin the motor driver ENA1 is attached to (PWM Pin)
const uint8_t m1_driver_in1_pin = 48;             // the pin the motor driver IN1 is attached to
const uint8_t m1_driver_in2_pin = 49;             // the pin the motor driver IN2 is attached to

//Pins for Motor encoder 
const uint8_t m1_encoder1_pin = 2;             // Motor encoder channel C1 (typically the Interrupt pin)
const uint8_t m1_encoder2_pin = 3;             // Motor encoder channel C2


//Pins for Battery Monitor / DIP Switch / LED
const uint8_t battery_monitor_pin = A3;         // Analog Pin

// ---- END OF PIN ASSIGNMENT  ----

//Tunings for motors
const double m1_kp = 0.005;                 // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile m1_kp = 0.040
const double m1_ki = 0.003;                 // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile m1_ki = 0.200
const double m1_kd = 0.0001;                // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile m1_kd = 0.0002

const double default_velocity = 500;			// Conservative speed
const double default_accel = 5000;               // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile m1_accel = 3000
const double default_error_to_stop = 200.0;         // Maximum absolute step error for the controller to stop itself without reaching the goal.
const long default_home_position_step = 0;
const double default_power = 1.0;			// Default to full power
const int motor_run_interval = 10;          // Motor PID sample Interval in millis()


// ---- END OF MODIFIABLE SETTINGS - Do not modify below ----

// Initialize motion control objects
DCMotor Motor1(m1_driver_ena_pin, m1_driver_in1_pin, m1_driver_in2_pin);
Encoder Encoder1(m1_encoder1_pin, m1_encoder2_pin);
MotorController MotorController1(&Motor1, &Encoder1, m1_kp, m1_ki, m1_kd, default_accel, motor_run_interval, default_error_to_stop, false, false);
long last_movement_start_time = 0;

// Variables for serial communication
byte incomingByte;
char status_string[61];

// Variables for battery monitor
int batt_value;

// Variables for communications
BufferedSerial bufferedSerial(1);

// Variables for profiling
unsigned long profile_start_micros = 0;
unsigned long profile_end_micros = 0;


// the setup function runs once when you press reset or power the board
void setup() {

	//Load settings
	loadSettings();

	// Initialize Serial
	bufferedSerial.serialInit();

	// Initialize battery monitor pin
	pinMode(battery_monitor_pin, INPUT);

}


// Load persistent settings from EEPROM
// This function must be run after MotorController is created
void loadSettings() {
	// Load homed position Setting - o
	long home_position_step = 0;
	EEPROM.get(10, home_position_step);
	MotorController1.setHomingParam(0, HIGH, home_position_step);
	// Load velocity Setting - v
	double velocity = 0.0;
	EEPROM.get(20, velocity);
	MotorController1.setDefaultVelocity(velocity);
	// Load accel Setting - a
	double accel = 0.0;
	EEPROM.get(30, accel);
	MotorController1.setAcceleration(accel);
	// Load error-to-stop  Setting
	double errorToStop = 0.0;
	EEPROM.get(40, errorToStop);
	MotorController1.setErrorToStop(errorToStop);
	// Load power Setting
	double maxPower = 0.0;
	EEPROM.get(50, maxPower);
	MotorController1.setMaxPower(maxPower);
}

const int setting_addr_o = 10;
const int setting_addr_v = 20;
const int setting_addr_a = 30;
const int setting_addr_e = 40;
const int setting_addr_p = 50;

void resetEEPROM() {
	EEPROM.put(setting_addr_o, default_home_position_step); // Reset homed position Setting
	EEPROM.put(setting_addr_v, default_velocity); // Reset velocity Setting
	EEPROM.put(setting_addr_a, default_accel); // Reset accel Setting
	EEPROM.put(setting_addr_e, default_error_to_stop); // Reset error-to-stop  Setting
	EEPROM.put(setting_addr_p, default_power); // Reset power Setting
}


void loop() {
	//The main loop implements quasi-time sharing task management.
	//This require all the subroutines to execute in relatively short time.
	//Long subroutine such as Serial prints should be used with cuation.


	// Run motor control
	if (MotorController1.run()) {
#if defined(SerialComment)
		Serial.print(F("CurPos: "));
		Serial.print((long)MotorController1.currentPosition());
		Serial.print(F(" Error: "));
		Serial.print((long)MotorController1.currentTarget() - (long)MotorController1.currentPosition());
		Serial.print(F(" PWM: "));
		Serial.println(MotorController1.currentMotorPowerPercentage());
#endif
	}

	// Run battery report
	run_batt_monitor();

	// Handle serial command input
	if (bufferedSerial.available()) {
		const char* command = bufferedSerial.read();
		//Serial.println(command); //Echo the received command
		run_command_handle(command);
	}

	// Reposrt status regularly
	run_motion_status_report();

}



// Serial and command parsing
// Input: command: pointer to a null terminated char array that holds the command string
void run_command_handle(const char* command) {

	if (*command == '?') {
		Serial.println(get_current_status_string());
	}

	// Action Command

	if (*command == 'h') {
		Serial.println(F("Command Home : Reset Motor Position to Zero"));
		MotorController1.resetEncoderPos();
	}


	if (*command == 'g') {
		long target_position_step = atol(command + 1);
		Serial.print(F("Goto Position:"));
		Serial.println(target_position_step);
		last_movement_start_time = millis();
		MotorController1.moveToPosition(target_position_step);
	}

	if (*command == 's') {
		Serial.println(F("Command s : Stop Now"));
		MotorController1.stop();
	}

	// Setting Command

	if (*command == 'o') {
		long home_position_step = atol(command + 1);
		Serial.print(F("Set Homed Position Offset:"));
		Serial.println(home_position_step);
		MotorController1.setHomingParam(0, HIGH, home_position_step);
		EEPROM.put(setting_addr_o, home_position_step); // Save new settings to EEPROM
	}

	if (*command == 'v') {
		double velocity = atof(command + 1);
		Serial.print(F("Set Velocity: "));
		Serial.println(velocity);
		MotorController1.setDefaultVelocity(velocity);
		EEPROM.put(setting_addr_v, velocity); // Save new settings to EEPROM
	}

	if (*command == 'a') {
		double accel = atof(command + 1);
		Serial.print(F("Set Acceleration: "));
		Serial.println(accel);
		MotorController1.setAcceleration(accel);
		EEPROM.put(setting_addr_a, accel); // Save new settings to EEPROM
	}

	if (*command == 'e') {
		double errorToStop = atof(command + 1);
		Serial.print(F("Set Error-To-Stop: "));
		Serial.println(errorToStop);
		MotorController1.setErrorToStop(errorToStop);
		EEPROM.put(setting_addr_e, errorToStop); // Save new settings to EEPROM
	}

	if (*command == 'p') {
		double max_power_level = atof(command + 1);
		if (max_power_level >= 0.0 && max_power_level <= 100.0) {
			Serial.print(F("Set Max Power Level: "));
			Serial.println(max_power_level);
			MotorController1.setMaxPower(max_power_level / 100);
			EEPROM.put(setting_addr_p, max_power_level / 100); // Save new settings to EEPROM
		}
		else {
			Serial.print(F("Error: Max Power must be between 0.0 to 100.0, received: "));
			Serial.println(max_power_level);
		}

	}

	if (*command == 'k') {
		if (*(command + 1) == 'p') {
			Serial.print(F("Set PID Kp: "));
			double value = atof(command + 2);
			MotorController1.setKp(value);
			Serial.println(value, 6);
		}
		if (*(command + 1) == 'i') {
			Serial.print(F("Set PID Ki:"));
			double value = atof(command + 2);
			MotorController1.setKi(value);
			Serial.println(value, 6);
		}
		if (*(command + 1) == 'd') {
			Serial.print(F("Set PID Kd:"));
			double value = atof(command + 2);
			MotorController1.setKd(value);
			Serial.println(value, 6);
		}
	}

	if (*command == 'x') {
		if (*(command + 1) == '1') {
			Serial.println(F("EEPROM Settings reset to default"));
			resetEEPROM();
		}
	}
	//if (*command == '+') {
//    long target_position_step = MotorController1.currentPosition() + atol(command + 1);
//    Serial.print("Increment Position:");
//    Serial.println(target_position_step);
//    MotorController1.moveToPosition(target_position_step);
//}

//if (*command == '-') {
//    long target_position_step = MotorController1.currentPosition() - atol(command + 1);
//    Serial.print("Decrement Position:");
//    Serial.println(target_position_step);
//    MotorController1.moveToPosition(target_position_step);
//}

}

void run_motion_status_report() {
	const unsigned long MONITOR_PEIROD_MILLIS = 30;
	static unsigned long next_run_time = 0;
	static boolean active = false;
	if (MotorController1.isMotorRunning()) {
		active = true;
	}
	if (active && (millis() > next_run_time)) {
		Serial.println(get_current_status_string());
	}
	if (not MotorController1.isMotorRunning()) {
		active = false;
	}
}

// Battery Monitor - To be separated into its own class and file.
void run_batt_monitor() {
	{
		const unsigned long MONITOR_PEIROD_MILLIS = 500;
		static unsigned long next_run_time = 0;
		if (millis() > next_run_time) {
			//profile_start();
			next_run_time = millis() + MONITOR_PEIROD_MILLIS;
			batt_value = analogRead(battery_monitor_pin);
			//profile_end("Batt Monitor Time Taken : ");
		}

	}
}


// Status Reporting
char* get_current_status_string() {
	unsigned int i = 0; // This variable keep count of the output string.
	i += snprintf(status_string + i, 60 - i, ">%u,%ld,%ld,%ld,%d,%i",
		get_status_code(),
		millis() - last_movement_start_time,
		(long)MotorController1.currentPosition(),
		(long)MotorController1.currentTarget(),
		(int)(MotorController1.currentMotorPowerPercentage() * 100.0),
		batt_value
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
