/*
 Name:		06_SL3_Controller.ino
 Created:	2021-05-03
 Author:	leungp


 This controller continues the development from 03_TokyoController
 Controls 1 main motor (m1) and 2 auxiliary motors (m2, m3 for grippers)

 The controller can accept commands from USB Serial (\n termainated)
 Radio commands can also be accepted. (This controller's default address is '1')
 h              - (Not really a Homing) Reset the axis position to zero
 ?              - Print out status
 g[position]    - move to a given position (step), can be positive or negative value.
 i[0/1]			- Retract / Extend Gripper
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
 - Main motor do not have homing switch
 - Gripper motor (2x) control and status feedback
 - Designed for Ardhuino Mega with more IO pins
 - DIP Switch now connectes to 4 pins instead of resistor ladder

 The behavior of the gripper motors are as follows:
 i0 - Retract (Loosen) Gripper Motors (full power)
	- Both motor will move towards the limit switch with the same speed. 
	- Individually, upon reaching the limit switch, the motor will stop.
	- If the motion profile is completed before reaching the switch, the retract is failed.
	- At retract-failed state, user can retry the retract but cannot initiate a tighten.

 i1 - Extend (Tighten) Gripper Motors (at lower power)
	- Both motors tightens (away from limit switch) with same motion profile
		this maintains sync position for the two pins.
	- Upon one of the motor jamming, the other motor will continue until it is also jammed, 
		or stop after distance-difference reach a threshold
	- If both motor completed their profile before reaching any jam, the extend is failed.
	- If any motor completed their profile too soon, the extend is failed.
	- At extend-fail state, user can retry extend or retract.
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
#include "states.h" // A neighbour file holding the constants defining states

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

//Tunings for main motor (28428 steps per rev)
const double m1_kp = 0.005;                 // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile m1_kp = 0.040
const double m1_ki = 0.003;                 // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile m1_ki = 0.200
const double m1_kd = 0.0001;                // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile m1_kd = 0.0002

//Tunings for gripper motors (
const double m2_kp = 0.005;                 // 
const double m2_ki = 0.003;                 // 
const double m2_kd = 0.0001;                // 

const double m3_kp = 0.005;                 // 
const double m3_ki = 0.003;                 // 
const double m3_kd = 0.0001;                // 

// Settings for pin gripper motors
const long gripper_max_extend_steps = 125000; // Full travel of 47mm (1.75pitch = 26.86rev) is 117589 steps
const long gripper_min_extend_steps = 25000; // Minimal amount of steps travelled before extend is considered successful.
const long gripper_retract_overshoot = 5000; // Position to aim at when going back to the switch,
											 // this can be zero but slight overshoot make sense.
const double gripper_velocity = 3000;			// Gripper motor 4378 steps per rev / pitch 1.75
const double gripper_accel = 5000;
const double gripper_error_to_stop = 1000.0;
const double gripper_extend_power = 0.75;
const double gripper_retract_power = 1.0;

// Default values when resetting EEPROM settings
const double default_velocity = 500;			// Conservative speed
const double default_accel = 5000;               // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile m1_accel = 3000
const double default_error_to_stop = 400.0;         // Maximum absolute step error for the controller to stop itself without reaching the goal.
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

unsigned int gripper_state = GRIPPER_NotHomed;
bool gripper_movement_success = false;
boolean gripper_sync_stop_countdown = false; // State of the sync stop checking routine. If true, the gripper is scheduled to stop.
unsigned long gripper_sync_stop_end_time = 0;

// Variables for serial communication
byte incomingByte;
char status_string[61];

// Variables for battery monitor
int batt_value;

// Variables for communications
BufferedSerial bufferedSerial(1);
CC1101 radio;
uint8_t radio_partnum, radio_version, radio_marcstate;	// Stores the register values of the radio fetched at startup.
boolean radio_found = false;
unsigned long radio_last_receive_millis = 0;
unsigned long radio_unfrozen_applied_millis = 0;
boolean radio_fix_enabled = true;
boolean serial_printout_enabled = false;

// Variables for profiling
unsigned long profile_start_micros = 0;
unsigned long profile_end_micros = 0;


// the setup function runs once when you press reset or power the board
void setup() {

	//Load settings
	loadMotorSettings();

	// Initialize Serial
	bufferedSerial.serialInit();

	// Initialize battery monitor pin
	pinMode(battery_monitor_pin, INPUT);

	// Initialize status LED pin
	pinMode(grn_led_pin, OUTPUT);
	pinMode(org_led_pin, OUTPUT);

	// Read DPI Switch and set Radio Address
	unsigned int DIPValue = 0;
	pinMode(dip_switch_pin_1, INPUT_PULLUP);
	pinMode(dip_switch_pin_2, INPUT_PULLUP);
	pinMode(dip_switch_pin_3, INPUT_PULLUP);
	pinMode(dip_switch_pin_4, INPUT_PULLUP);
	DIPValue += 1 - digitalRead(dip_switch_pin_1) << 0;
	DIPValue += 1 - digitalRead(dip_switch_pin_2) << 1;
	DIPValue += 1 - digitalRead(dip_switch_pin_3) << 2;
	DIPValue += 1 - digitalRead(dip_switch_pin_4) << 3;


	Serial.print(F("(DIPValue = "));
	Serial.print(DIPValue);
	Serial.print(F(" This radio address is set to : char "));
	//Truth table please refer to 03_TokyoController.md
	// SW1, SW2, SW3 = 0 results in address '1'
	Serial.print(radio_selfAddress = DIPValue + 48 + 1);
	Serial.println(")");

	// Start Radio
	if (RadioStartup()) {
		Serial.println(F("(CC1101 Radio Startup OK)"));
	}
	else {
		Serial.println(F("(CC1101 Radio Startup Error Radio Abnormal)"));
	}

	Serial.println(F("Serial printout disabled by default, send any command via Serial to activiate it."));
}

// Routine to start Radio
// Return true when startup is successful
boolean RadioStartup() {
	radio.init();
	radio.setSyncWord(radio_syncWord[0], radio_syncWord[1]);
	radio.setDevAddress(radio_selfAddress);
	radio.setCarrierFreq(frequency);
	radio.setTxPowerAmp(radio_power);
	radio.setChannel(channelNumber);
	radio.setRxState();

	delay(50);

	radio_partnum = radio.readReg(CC1101_PARTNUM, CC1101_STATUS_REGISTER);
	radio_version = radio.readReg(CC1101_VERSION, CC1101_STATUS_REGISTER);
	radio_marcstate = radio.readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1f;
	auto radio_CC1101_IOCFG0_value = radio.readReg(CC1101_IOCFG0, CC1101_CONFIG_REGISTER);
	auto radio_CC1101_PKTCTRL1_value = radio.readReg(CC1101_PKTCTRL1, CC1101_CONFIG_REGISTER);

	// Determine if radio is found. If everything is zero, the radio is probably dead.
	if (radio_partnum == 0 && radio_version == 0 && radio_marcstate == 0) {
		// Radio Not Found - report to Serial
#if defined(SerialComment)
		Serial.println(F("(CC1101 radio abnormal)"));
#endif
		radio_found = false;
		return false;
	}
	else {
		// Radio Found - Report to Serial
#if defined(SerialComment)
		Serial.println(F("(CC1101 radio ok)"));
		Serial.print(F("(CC1101_PARTNUM = "));
		Serial.print(radio_partnum);
		Serial.print(F(" , CC1101_VERSION "));
		Serial.print(radio_version);
		Serial.print(F(" , CC1101_MARCSTATE "));
		Serial.print(radio_marcstate);
		Serial.print(F(" , CC1101_IOCFG0 "));
		Serial.print(radio_CC1101_IOCFG0_value);
		Serial.print(F(" , CC1101_PKTCTRL1 "));
		Serial.print(radio_CC1101_PKTCTRL1_value);
		Serial.println(F(") (Typ Values: 0,20,13,7,14)"));
#endif
		radio_found = true;
		return true;
	}

}


const int setting_addr_o = 10;
const int setting_addr_v = 20;
const int setting_addr_a = 30;
const int setting_addr_e = 40;
const int setting_addr_p = 50;

// Load persistent settings from EEPROM
// This function must be run after MotorController is created
void loadMotorSettings() {
	// Load homed position Setting - o
	long home_position_step = 0;
	EEPROM.get(setting_addr_o, home_position_step);
	MotorController1.setHomingParam(0, HIGH, home_position_step);
	// Load velocity Setting - v
	double velocity = 0.0;
	EEPROM.get(setting_addr_v, velocity);
	MotorController1.setDefaultVelocity(velocity);
	// Load accel Setting - a
	double accel = 0.0;
	EEPROM.get(setting_addr_a, accel);
	MotorController1.setAcceleration(accel);
	// Load error-to-stop  Setting
	double errorToStop = 0.0;
	EEPROM.get(setting_addr_e, errorToStop);
	MotorController1.setErrorToStop(errorToStop);
	// Load power Setting
	double maxPower = 0.0;
	EEPROM.get(setting_addr_p, maxPower);
	MotorController1.setMaxPower(maxPower);

	// Gripper Motor Settings  // Fixed settings, not loading from EEPROM
	MotorController2.setHomingParam(m2_home_pin, HIGH, 0);
	MotorController2.setDefaultVelocity(gripper_velocity);
	MotorController2.setAcceleration(gripper_accel);
	MotorController2.setErrorToStop(gripper_error_to_stop);
	MotorController3.setHomingParam(m3_home_pin, HIGH, 0);
	MotorController3.setDefaultVelocity(gripper_velocity);
	MotorController3.setAcceleration(gripper_accel);
	MotorController3.setErrorToStop(gripper_error_to_stop);

}


void resetEEPROM() {
	EEPROM.put(setting_addr_o, default_home_position_step); // Reset homed position Setting
	EEPROM.put(setting_addr_v, default_velocity); // Reset velocity Setting
	EEPROM.put(setting_addr_a, default_accel); // Reset accel Setting
	EEPROM.put(setting_addr_e, default_error_to_stop); // Reset error-to-stop  Setting
	EEPROM.put(setting_addr_p, default_power); // Reset power Setting
}

void loop() {
	//The main loop implements quasi-time sharing task management.
	// Higher tasks have higher priority
	//This require all the subroutines to execute in relatively short time.
	//Long subroutine such as Serial prints should be used with cuation.


	// Run Main motor control
	if (MotorController1.run()) {
#if defined(SerialComment)
		Serial.print(F("CurPos: "));
		Serial.print((long)MotorController1.currentPosition());
		Serial.print(F(" Error: "));
		Serial.print((long)MotorController1.currentTarget() - (long)MotorController1.currentPosition());
		Serial.print(F(" PWM: "));
		Serial.println(MotorController1.currentMotorPowerPercentage());
#endif
		//return;
	}



	// Run Gripper motor
	if (MotorController2.run()) {
		Serial.print("Motor 2 Running. Pos:");
		Serial.print((long)MotorController2.currentPosition());
		Serial.print(" Power:");
		Serial.println((int)(MotorController2.currentMotorPowerPercentage() * 100.0));
		//return;
	}
	if (MotorController3.run()) {
		Serial.print("Motor 3 Running. Pos:");
		Serial.print((long)MotorController3.currentPosition());
		Serial.print(" Power:");
		Serial.println((int)(MotorController3.currentMotorPowerPercentage() * 100.0));
		//return;
	}

	// Run gripper motor sync stop
	if (gripper_motor_sync_stop()) {
		if (serial_printout_enabled) Serial.println("Gripper Motor Out of Sync Stopped.");
		return;
	}
	//static bool _m2_ran = MotorController2.run();
	//static bool _m3_ran = MotorController3.run();
	//if (_m2_ran || _m3_ran) return;



	// Handle serial command input
	if (bufferedSerial.available()) {
		const char* command = bufferedSerial.read();
		serial_printout_enabled = true;
		Serial.println(command); //Echo the received command
		run_command_handle(command);
		return;
	}

	// Handle radio command input
	if (radioAvailable()) {
		CCPACKET packet;
		radio.receiveData(&packet);
		// Null terminate the packet data because by default it is not.
		// Command_handle expects the (char *) to be null terminated.
		packet.data[packet.length] = 0;

#if defined(SerialComment)
		Serial.print(F("Radio PKT RX: "));
		for (unsigned int i = 0; i < packet.length; i++) {
			Serial.write(packet.data[i]);
		}
		Serial.print(F(" (LQI = "));
		Serial.print(lqi(packet.lqi));
		Serial.print(F(" , RSSI = "));
		Serial.print(rssi(packet.rssi));
		Serial.println(F(")"));
#endif

		// Pass this to command  // Skip the first two addresses
		run_command_handle((char*)packet.data + 2);

		// Write Status Back
		CCPACKET packet_reply;
		get_current_status_string();
		//Serial.println(get_current_status_string());
		packet_reply.length = strlen(status_string) + 2; // status string is a null terminated string.
		packet_reply.data[0] = radio_master_address;
		packet_reply.data[1] = radio_selfAddress;
		strcpy((char*)packet_reply.data + 2, status_string);
		radio.sendDataSpecial(packet_reply);

		// Record the last Radio Reception time
		radio_last_receive_millis = millis();
		return;
	}
	
	// Run Radio anti-freeze
	if (run_radio_frozen_fix()) return;

	// Run battery report
	if (run_batt_monitor()) return;


	run_status_light();

}

boolean radioAvailable() {
	return digitalRead(radio_gdo0_pin);
}

int lqi(char raw) {
	return 0x3F - raw;
}

int rssi(char raw) {
	uint8_t rssi_dec;
	// TODO: This rssi_offset is dependent on baud and MHz; But for cc1101, it is constantly 74. see DN505
	uint8_t rssi_offset = 74;
	rssi_dec = (uint8_t)raw;
	if (rssi_dec >= 128)
		return ((int)(rssi_dec - 256) / 2) - rssi_offset;
	else
		return (rssi_dec / 2) - rssi_offset;
}


// Serial and command parsing
// Input: command: pointer to a null terminated char array that holds the command string
void run_command_handle(const char* command) {

	if (*command == '?') {
		if (serial_printout_enabled) Serial.println(get_current_status_string());
	}

	// Action Command

	if (*command == 'h') {
		if (serial_printout_enabled) Serial.println(F("Command Home : Reset Main Motor Position"));
		MotorController1.resetEncoderPos();
		MotorController2.home(true, gripper_velocity, 117500);
		MotorController3.home(true, gripper_velocity, 117500);
		gripper_state = GRIPPER_Retracting;
	}


	if (*command == 'g') {
		long target_position_step = atol(command + 1);
		if (serial_printout_enabled) Serial.print(F("Goto Position:"));
		if (serial_printout_enabled) Serial.println(target_position_step);
		MotorController1.moveToPosition(target_position_step);
	}

	if (*command == 's') {
		if (serial_printout_enabled) Serial.println(F("Command s : Stop Now"));
		MotorController1.stop();
		MotorController2.stop();
		MotorController3.stop();

	}

	if (*command == 'i') {
		if (*(command + 1) == '0') {
			if (serial_printout_enabled) Serial.println(F("Command i0 : Retract Gripper Pins"));
			gripper_retract();
		}

		if (*(command + 1) == '1') {
			if (serial_printout_enabled) Serial.println(F("Command i1 : Extend Gripper Pins"));
			gripper_extend();
		}
	}

	if (*command == 'o') {
		long home_position_step = atol(command + 1);
		if (serial_printout_enabled) Serial.print(F("Set Homed Position Offset:"));
		if (serial_printout_enabled) Serial.println(home_position_step);
		MotorController1.setHomingParam(0, HIGH, home_position_step);
		EEPROM.put(setting_addr_o, home_position_step); // Save new settings to EEPROM
	}

	if (*command == 'v') {
		double velocity = atof(command + 1);
		if (serial_printout_enabled) Serial.print(F("Set Velocity: "));
		if (serial_printout_enabled) Serial.println(velocity);
		MotorController1.setDefaultVelocity(velocity);
		EEPROM.put(setting_addr_v, velocity); // Save new settings to EEPROM
	}

	if (*command == 'a') {
		double accel = atof(command + 1);
		if (serial_printout_enabled) Serial.print(F("Set Acceleration: "));
		if (serial_printout_enabled) Serial.println(accel);
		MotorController1.setAcceleration(accel);
		EEPROM.put(setting_addr_a, accel); // Save new settings to EEPROM
	}

	if (*command == 'e') {
		double errorToStop = atof(command + 1);
		if (serial_printout_enabled) Serial.print(F("Set Error-To-Stop: "));
		if (serial_printout_enabled) Serial.println(errorToStop);
		MotorController1.setErrorToStop(errorToStop);
		EEPROM.put(setting_addr_e, errorToStop); // Save new settings to EEPROM
	}

	if (*command == 'p') {
		double max_power_level = atof(command + 1);
		if (max_power_level >= 0.0 && max_power_level <= 100.0) {
			if (serial_printout_enabled) Serial.print(F("Set Max Power Level: "));
			if (serial_printout_enabled) Serial.println(max_power_level);
			MotorController1.setMaxPower(max_power_level / 100);
			EEPROM.put(setting_addr_p, max_power_level / 100); // Save new settings to EEPROM
		}
		else {
			if (serial_printout_enabled) Serial.print(F("Error: Max Power must be between 0.0 to 100.0, received: "));
			if (serial_printout_enabled) Serial.println(max_power_level);
		}

	}


	if (*command == 'x') {
		if (*(command + 1) == '1') {
			if (serial_printout_enabled) Serial.println(F("EEPROM Settings reset to default"));
			resetEEPROM();
		}
	}

	if (*command == 'f') {
		if (*(command + 1) == '0') {
			if (serial_printout_enabled) Serial.println(F("Command f : Radio Fix Disabled"));
			radio_fix_enabled = false;
		}
		if (*(command + 1) == '1') {
			if (serial_printout_enabled) Serial.println(F("Command f : Radio Fix Enabled"));
			radio_fix_enabled = true;
		}
	}

	// For testing
	if (*command == 'r') {
		if (serial_printout_enabled) Serial.println(F("Command r : Send Test Radio Message to Master"));
		// Write Status Back
		CCPACKET packet_reply;

		// Copy the rest of the command to send it
		strcpy((char*)packet_reply.data + 1, command + 1);
		packet_reply.length = strlen((char*)packet_reply.data);
		packet_reply.data[0] = radio_master_address;

		auto result = radio.sendDataSpecial(packet_reply);
		if (serial_printout_enabled) Serial.println((char*)&packet_reply.data);
		if (result && serial_printout_enabled) Serial.println(F("Sent Succeed"));

	}
}

//Both motor will move towards the limit switch with the same speed.
//Upon reaching the limit switch, the motor will stop. Similar to a homing cycle.
void gripper_retract() {
	MotorController2.setMaxPower(gripper_retract_power);
	MotorController3.setMaxPower(gripper_retract_power);
	// If motor have never been homed, it will home full travel
	if (gripper_state == GRIPPER_NotHomed) {
		MotorController2.home(true, gripper_velocity, gripper_max_extend_steps);
		MotorController3.home(true, gripper_velocity, gripper_max_extend_steps);
	}
	// If motor is homed before. The homing amount is = current position (+ve) + overshoot (+ve)
	else {
		MotorController2.home(true, gripper_velocity, MotorController2.currentPosition() + gripper_retract_overshoot);
		MotorController3.home(true, gripper_velocity, MotorController3.currentPosition() + gripper_retract_overshoot);
	}

	gripper_state = GRIPPER_Retracting;
	gripper_sync_stop_countdown = false;
}

//Both gripper motors tightens (away from limit switch) with same motion profile
//this maintains sync position for the two pins. 
//Gripper must be homed (retracted) once after power on to be extended.
void gripper_extend() {
	if (gripper_state == GRIPPER_NotHomed) {
		if (serial_printout_enabled) Serial.println(F("Cannot Extend Gripper because GRIPPER_NotHomed."));
		return;
	}
	MotorController2.setMaxPower(gripper_extend_power);
	MotorController3.setMaxPower(gripper_extend_power);
	MotorController2.moveToPosition(gripper_max_extend_steps);
	MotorController3.moveToPosition(gripper_max_extend_steps);
	gripper_state = GRIPPER_Extending;
	gripper_sync_stop_countdown = false;
}

// Gripper motor sync stop check. 
// Returns true if the gripper_state is changed.
boolean gripper_motor_sync_stop() {
	//if (serial_printout_enabled) Serial.print(F("Checking Sync."));
	if (!(gripper_state == GRIPPER_Extending || gripper_state == GRIPPER_Retracting)) return false;

	// Retracting
	if (gripper_state == GRIPPER_Retracting) {
		if (!MotorController2.isMotorRunning() && !MotorController3.isMotorRunning()) {
			if (!MotorController2.isHomed() || !MotorController3.isHomed()) {
				gripper_state = GRIPPER_RetractFail;
				if (serial_printout_enabled) {
					Serial.print(F("Gripper Retract Fail. Motor2 Pos:"));
					Serial.print(MotorController2.currentPosition());
					Serial.print(F(" Motor3 Pos:"));
					Serial.println(MotorController3.currentPosition());
				}
				gripper_movement_success = false;
				return true;
			}
			else {
				gripper_state = GRIPPER_Retracted;
				if (serial_printout_enabled) {
					Serial.println(F("Gripper Retract Success."));
				}
				gripper_movement_success = true;
				return true;
			}
		}
	};
	//if (serial_printout_enabled) Serial.print(F("Checking Sync.2"));

	// Extending
	if (gripper_state == GRIPPER_Extending) {

		// Success if both motors are stopped by jamming
		if (!MotorController2.isMotorRunning() && !MotorController3.isMotorRunning()) {
			if (serial_printout_enabled) Serial.println(F("Both Gripper Extension Stopped."));
			
			if (!MotorController2.isTargetReached() && !MotorController3.isTargetReached()) {
				if (serial_printout_enabled) {
					Serial.println(F("Gripper Extend Success. Motor2 Pos:"));
					Serial.print(MotorController2.currentPosition());
					Serial.println(F(" Motor3 Pos:"));
					Serial.println(MotorController3.currentPosition());
				}
				gripper_state = GRIPPER_Extended;
				gripper_movement_success = true;
			}
			else {
				if (serial_printout_enabled) {
					if (MotorController2.isTargetReached()) {
						Serial.println(F("Gripper Extend Fail. Motor 2 Reached Target without jam"));
					}
					if (MotorController3.isTargetReached()) {
						Serial.println(F("Gripper Extend Fail. Motor 3 Reached Target without jam"));
					}
				}
				gripper_state = GRIPPER_ExtendFail;
				gripper_movement_success = false;
			}
			return true;
		}

		//- If both motor completed their profile before reaching any jam, or stopped too soon the extend is failed.
		if (!MotorController2.isMotorRunning()){
			if (MotorController2.currentPosition() < gripper_min_extend_steps) {
				if (serial_printout_enabled) {
					Serial.println(F("Gripper Extend Fail. Motor 2 jammed before extending minimum number of steps. Pos:"));
					Serial.println(MotorController2.currentPosition());
				}
				MotorController3.stop();
				gripper_state = GRIPPER_ExtendFail;
				gripper_movement_success = false;
				return true;
			}
		}
		if (!MotorController3.isMotorRunning()) {
			if (MotorController3.currentPosition() < gripper_min_extend_steps) {
				if (serial_printout_enabled) {
					Serial.println(F("Gripper Extend Fail. Motor 3 jammed before extending minimum number of steps. Pos:"));
					Serial.println(MotorController3.currentPosition());
				}
				MotorController2.stop();
				gripper_state = GRIPPER_ExtendFail;
				gripper_movement_success = false;
				return true;
			}
		}

	}

	// Setting Command
	return false;
};

// Battery Monitor - To be separated into its own class and file.
// Return true if battery is checked.

boolean run_batt_monitor() {
	{
		const unsigned long MONITOR_PEIROD_MILLIS = 500;
		static unsigned long next_run_time = 0;
		if (millis() > next_run_time) {
			//profile_start();
			next_run_time = millis() + MONITOR_PEIROD_MILLIS;
			batt_value = analogRead(battery_monitor_pin);
			//profile_end("Batt Monitor Time Taken : ");
			return true;
		}
		return false;
	}
}

// Quick Fix to deal with radio occationally frozen.
// Return true if fix has been applied.
boolean run_radio_frozen_fix() {
	// radio_frozen_fix can be turned off via serial command.
	if (!radio_fix_enabled) return false;

	const unsigned long RADIO_FROZEN_TIMEOUT = 3000;
	// If radio do not receive anything within timeout, radio will reset itself into RX mode.
	if (millis() > radio_last_receive_millis + RADIO_FROZEN_TIMEOUT) {
		if (millis() > radio_unfrozen_applied_millis + RADIO_FROZEN_TIMEOUT) {
			radio.setRxState();
			//radio.flushRxFifo();
			//radio.flushTxFifo();
			//if (serial_printout_enabled) Serial.println(F("RadioFixApplied"));
			radio_unfrozen_applied_millis = millis();
			return true;
		}
	}
	return false;
}

// Status Reporting
char* get_current_status_string() {
	unsigned int i = 0; // This variable keep count of the output string.
	i += snprintf(status_string + i, 60 - i, "%u,%ld,%ld,%d,%i,%ld,%ld,%ld,%ld,%i",
		get_status_code(),
		(long)MotorController1.currentPosition(),
		(long)MotorController1.currentTarget(),
		(int)(MotorController1.currentMotorPowerPercentage() * 100.0),
		batt_value,
		(long)MotorController2.currentPosition(),
		(long)MotorController2.currentTarget(),
		(long)MotorController3.currentPosition(),
		(long)MotorController3.currentTarget(),
		get_gripper_state_code()
	);
	return status_string;
}

int get_gripper_state_code() {
	if (gripper_state == GRIPPER_NotHomed) return GRIPPER_NotHomed;
	if (MotorController2.isMotorRunning() || MotorController3.isMotorRunning()) {
		if (MotorController2.isDirectionExtend()) {
			return GRIPPER_Extending;
		}
		else {
			return GRIPPER_Retracting;
		}
	}
	else {
		if (gripper_movement_success) {
			if (MotorController2.isDirectionExtend()) return GRIPPER_Extended; else return GRIPPER_Retracted;
		}
		else {
			if (MotorController2.isDirectionExtend()) return GRIPPER_ExtendFail; else return GRIPPER_RetractFail;
		}
	}

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

		// ** Blue Status Light (Communication)**
		//- Long Lit = Communication received in past 1 sec
		//- Blink = Communication not received for over 1 sec.
		//- Off = Radio Bad
		

		const unsigned long RADIO_NO_RECEIVE_THRESHOLD = 1000;
		if (!radio_found) {
			digitalWrite(blu_led_pin, LOW);
		}
		else if (millis() - radio_last_receive_millis >= RADIO_NO_RECEIVE_THRESHOLD) {
			digitalWrite(blu_led_pin, blinking_high_low);
		}
		else {
			digitalWrite(blu_led_pin, HIGH);
		}

		// ** Orange Status Light **
		//- Long Lit = Main Motor (1) or Gripper Motor (2/3) Moving
		//- Blink = Gripper Motor Stuck
		//- Off = Standby / Normal

		if (MotorController1.isMotorRunning() || MotorController2.isMotorRunning() || MotorController3.isMotorRunning()) {
			digitalWrite(org_led_pin, HIGH);
		}
		// TODO Blink = Gripper Motor Stuck
		else {
			digitalWrite(org_led_pin, LOW);
		}

	}
}
