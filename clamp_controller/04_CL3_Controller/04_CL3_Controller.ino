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
 t[position]    - set/override curent position (step) without moving

 o[offset-pos]	- Set Homed Offset (step) (persistent) (NEW) 
 v[velocity]    - Set Velocity in (step/s) (persistent)
 a[accel]		- Set Acceleration in (step/s^2) (persistent)
 e[error]		- Set Error-To-Stop (steps) (persistent)
 p[power]       - Set Maximum Power used in motor control. (range 0 to 100) (persistent) (NEW)
 x[1]			- Reset settings stored in EEPROM to default values.

 r[message]     - Send radio message to master (default address '0') e.g. rHello\n
 f[0/1]			- Enable Disable Radio Fix

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

#include <DCMotor.h>        // clamp_firmware library
#include <Encoder.h>        // Encoder library from PJRC.COM, LLC - Paul Stoffregen http://www.pjrc.com/teensy/td_libs_Encoder.html
#include <MotorController.h>    //New Class in Development

#include <BufferedSerial.h>

#include <cc1101_nointerrupt.h>
#include <ccpacket.h>
#include <ResistorLadderID.h>

//#include <Wire.h>


// #define SerialComment
//MOTOR_CONTROL_PRINTOUT (if defined) will print motor control error during motor movements.
// #define MOTOR_CONTROL_PRINTOUT

//SERIAL_MASTER_INIT_TALK_ONLY (if defined) will ensure the controller do not initiate any communication.
//	The controller will act as a slave node and only talk back when the master initiate a message.
//	This is crucial in a master managed time share network to avoid bus contention.
//	This feature can be disabled for debug use when the controller is connected directly to a PC.
//#define SERIAL_MASTER_INIT_TALK_ONLY


//Pins for Motor Driver M1
const uint8_t m1_driver_ena_pin = 5;             // the pin the motor driver ENA1 is attached to (PWM Pin)
const uint8_t m1_driver_in1_pin = 4;             // the pin the motor driver IN1 is attached to
const uint8_t m1_driver_in2_pin = 7;             // the pin the motor driver IN2 is attached to

//Pins for Motor Driver M2
const uint8_t m2_driver_ena_pin = 6;          // Reserved Pin
const uint8_t m2_driver_in1_pin = 8;          // Reserved Pin
const uint8_t m2_driver_in2_pin = 9;          // Reserved Pin


//Pins for Motor encoder (One Motor)
const uint8_t m1_encoder1_pin = 2;             // Motor encoder channel C1 (typically the Interrupt pin)
const uint8_t m1_encoder2_pin = 3;             // Motor encoder channel C2

//Pins for Motor encoder (Two Motors)
//const uint8_t m1_encoder1_pin = 2;             // Motor encoder channel C1 (typically the Interrupt pin)
//const uint8_t m1_encoder2_pin = A4;            // Motor encoder channel C2
//const uint8_t m2_encoder1_pin = 3;             // Motor encoder channel C1 (typically the Interrupt pin)
//const uint8_t m2_encoder2_pin = A5;            // Motor encoder channel C2

//Pins for Homing Switch
const uint8_t m1_home_pin = A1;                 // INPUT_PULLUP Mode
//const uint8_t m2_home_pin = A2;                 // INPUT_PULLUP Mode

//Pins for radio
const uint8_t radio_ss_pin = 10;                // Hardware SPI Interface
const uint8_t radio_mosi_pin = 11;              // Hardware SPI Interface
const uint8_t radio_miso_pin = 12;              // Hardware SPI Interface
const uint8_t radio_sck_pin = 13;               // Hardware SPI Interface
const uint8_t radio_gdo0_pin = A0;              // Input to sense incoming package

//Pins for Battery Monitor / DIP Switch / LED
const uint8_t battery_monitor_pin = A7;         // Analog Pin
const uint8_t dip_switch_pin = A6;

const uint8_t grn_led_pin = A3;
const uint8_t org_led_pin = 6;

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
MotorController MotorController1(&Motor1, &Encoder1, m1_kp, m1_ki, m1_kd, default_accel, motor_run_interval, default_error_to_stop, false, false);

// Variables for serial communication
byte incomingByte;
char status_string[61];

// Variables for battery monitor
int batt_value;

// ResistorLadderID
const int addressValues[] = { 0, 73, 140, 198, 258, 302, 341, 376, 431, 457, 482, 504, 529, 547, 564, 580 };
ResistorLadderID DipSwitch(A6, addressValues, 16);

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


// the setup function runs once when you press reset or power the board
void setup() {

	//Load settings
	loadSettings();

	// Initialize Serial
	bufferedSerial.serialInit();

	// Initialize battery monitor pin
	pinMode(battery_monitor_pin, INPUT);

	// Initialize status LED pin
	pinMode(grn_led_pin, OUTPUT);
	pinMode(org_led_pin, OUTPUT);

	// Read DPI Switch and set Radio Address
	DipSwitch.init();
	unsigned int DIPValue = DipSwitch.read();
	Serial.print(F("(DIPValue = "));
	Serial.print(DIPValue);
	Serial.print(F(" This radio address is set to : char "));
	//Truth table please refer to 03_TokyoController.md
	// SW1, SW2, SW3 = 0 results in address '1'
	Serial.print(radio_selfAddress = (DIPValue >> 1) + 48 + 1);
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
		return false;
#if defined(SerialComment)
		Serial.println(F("(CC1101 radio abnormal)"));
#endif
	}
	else {
		return true;
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

	}

}

// Load persistent settings from EEPROM
// This function must be run after MotorController is created
void loadSettings() {
	// Load homed position Setting - o
	long home_position_step = 0;
	EEPROM.get(10, home_position_step);
	MotorController1.setHomingParam(m1_home_pin, HIGH, home_position_step);
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
		serial_printout_enabled = true;
		Serial.println(command); //Echo the received command
		run_command_handle(command);
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
	}

	run_radio_frozen_fix();
	run_status_light();

	//CCPACKET packet;
	//radio.receiveData(&packet);

	//if (packet.length > 0) {
	//    Serial.print(F("PACKET Recieved: "));
	//    for (unsigned int i = 0; i < packet.length; i++) {
	//        Serial.write(packet.data[i]);
	//    }
	//    Serial.print(F(" (LQI = "));
	//    Serial.print(lqi(packet.lqi));
	//    Serial.print(F(" , RSSI = "));
	//    Serial.print(rssi(packet.rssi));
	//    Serial.println(F(")"));
	//}

	//if (digitalRead(radio_gdo0_pin)) {
	//    Serial.print(F("GDO Is HIGH"));
	//}

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
		if (serial_printout_enabled) Serial.println(F("Command Home : Homing"));
		MotorController1.home(true, 1500, 200000);
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
	}

	if (*command == 't') {
		long target_position_step = atol(command + 1);
		if (serial_printout_enabled) Serial.print(F("Command Set Current Position to: "));
		if (serial_printout_enabled) Serial.println(target_position_step);
		MotorController1.resetEncoderPos(target_position_step);
	}
	// Setting Command

	if (*command == 'o') {
		long home_position_step = atol(command + 1);
		if (serial_printout_enabled) Serial.print(F("Set Homed Position Offset:"));
		if (serial_printout_enabled) Serial.println(home_position_step);
		MotorController1.setHomingParam(m1_home_pin, HIGH, home_position_step);
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

// Quick Fix to deal with radio occationally frozen.
void run_radio_frozen_fix() {
	// radio_frozen_fix can be turned off via serial command.
	if (!radio_fix_enabled) return;

	const unsigned long RADIO_FROZEN_TIMEOUT = 3000;
	// If radio do not receive anything within timeout, radio will reset itself into RX mode.
	if (millis() > radio_last_receive_millis + RADIO_FROZEN_TIMEOUT) {
		if (millis() > radio_unfrozen_applied_millis + RADIO_FROZEN_TIMEOUT) {
			radio.setRxState();
			//radio.flushRxFifo();
			//radio.flushTxFifo();
			if (serial_printout_enabled) Serial.println(F("RadioFixApplied"));
			radio_unfrozen_applied_millis = millis();
		}
	}

	
}

// Status Reporting
char* get_current_status_string() {
	unsigned int i = 0; // This variable keep count of the output string.
	i += snprintf(status_string + i, 60 - i, "%u,%ld,%ld,%d,%i",
		get_status_code(),
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
