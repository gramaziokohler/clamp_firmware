/*
 Name:		Motor08_PID_TrapezoidalMotionProfile.ino
 Created:	21/10/2019
 Author:	leungp

*/


#include "DCMotor.h"
#include "Encoder.h"

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

#define TUNE_MOTOR_NUM3

// ---- END OF PIN ASSIGNMENT  ----

const uint8_t driver_ena_pin = 9;             // the pin the motor driver ENA1 is attached to (PWM Pin)
const uint8_t driver_in1_pin = 8;             // the pin the motor driver IN1 is attached to
const uint8_t driver_in2_pin = 7;             // the pin the motor driver IN2 is attached to


#if defined TUNE_MOTOR_NUM1
DCMotor Motor(m1_driver_ena_pin, m1_driver_in1_pin, m1_driver_in2_pin);
Encoder myEnc(m1_encoder1_pin, m1_encoder2_pin);
#elif defined TUNE_MOTOR_NUM2
DCMotor Motor(m2_driver_ena_pin, m2_driver_in1_pin, m2_driver_in2_pin);
Encoder myEnc(m2_encoder1_pin, m2_encoder2_pin);
#elif defined TUNE_MOTOR_NUM3
DCMotor Motor(m3_driver_ena_pin, m3_driver_in1_pin, m3_driver_in2_pin);
Encoder myEnc(m3_encoder1_pin, m3_encoder2_pin);

#endif

//DCMotor Motor1(driver_ena_pin, driver_in1_pin, driver_in2_pin);



#include <PID_v1.h>

#include "MotionProfile.h"

void perform_one_test(double kp, double ki, double kd, double velocityStepsPerSec, double accelStepsPerSecSq, double preDurationSec, double runDurationSec, double postDuration_Sec) {
    //Print Octave matrix format header to start:
    String testResultName = "result_" + String(kp, 4);
    testResultName += "_" + String(ki, 4);
    testResultName += "_" + String(kd, 4);
    testResultName += "_" + String(velocityStepsPerSec, 0);
    testResultName += "_" + String(accelStepsPerSecSq, 0);
    testResultName += "_" + String(runDurationSec, 0);
    testResultName.replace('.', 'p');
    testResultName.replace(" ", "");
    Serial.print(testResultName);
    Serial.println(" = [");

    // Compute steps
    double total_steps = runDurationSec * velocityStepsPerSec;


    // Reset Encoder
    myEnc.write(0);

    // Setup new motion profile
    //LinearMotionProfile profile = LinearMotionProfile(0, total_steps, velocityStepsPerSec);
    TrapezoidalMotionProfile profile = TrapezoidalMotionProfile(0, total_steps, velocityStepsPerSec, accelStepsPerSecSq);

    //Serial.println("_phase1End_Micros=" + String(profile._phase1End_Micros));
    //Serial.println("_phase2End_Micros=" + String(profile._phase2End_Micros));
    //Serial.println("_phase3End_Micros=" + String(profile._phase3End_Micros));

    // Setup PID Positional Control Variables
    double current_position_step = 0;
    double target_position_step = 0;
    double motorSpeedPercentage = 0.0;

    // Create PID, Configure PID controller
    PID myPID(&current_position_step, &motorSpeedPercentage, &target_position_step, kp, ki, kd, DIRECT);
    myPID.SetOutputLimits(-1.0, 1.0);
    myPID.SetSampleTime(2);
    myPID.SetMode(1); //Turn on PID

    // Compute Duration
    long total_duration_micros = preDurationSec * 1e6  + profile.getTotalDurationMicros() + postDuration_Sec * 1e6 ;
    unsigned long testStartTimeMicros = micros();

    while (micros() - testStartTimeMicros < total_duration_micros) {
        if ((!profile.isStarted()) && ((micros() - testStartTimeMicros) > (preDurationSec * 1e6))) {
            // Start Motion Profile
            profile.start();
        }
        //Read encoder
        current_position_step = myEnc.read();

        //Read motion profile
        target_position_step = profile.getCurrentStep();

        //Compute PID
        myPID.Compute();

        //Set Motor PWM based on PID Output
        Motor.setSpeedPercent(motorSpeedPercentage);

        //Report
        reporting(5, (long)micros() - testStartTimeMicros - (preDurationSec * 1e6), target_position_step, current_position_step, motorSpeedPercentage);


    }

    //Finalize the test (stop motor if PID didn't stop it perfectly)
    Motor.stop();
    myPID.SetMode(0);

    //Print Octave matrix format footer to start:
    Serial.println("]; % " + testResultName);
}

void reporting(long ReportIntervalMillis, long time, double target_position_step, double current_position_step, double motorSpeedPercentage) {
    static unsigned long lastReportTime = 0;
    long deltaTime = millis() - lastReportTime;
    if (deltaTime > ReportIntervalMillis) {
        lastReportTime = millis();
        Serial.print(time);
        Serial.print(' ');
        Serial.print(target_position_step, 1);
        Serial.print(' ');
        Serial.print(current_position_step, 1);
        Serial.print(' ');
        Serial.print(motorSpeedPercentage);
        Serial.print('\n');
    }
}

void setup() {
    TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to 8 for PWM frequency of 3921.16 Hz
    Serial.begin(115200);
    Serial.setTimeout(10);
    Motor.setSpeedPercent(0.0);

    perform_one_test(0.040, 0.100, 0.0001, 3000, 5000, 0.2, 3.0, 1.0);
    delay(1000);
    perform_one_test(0.040, 0.080, 0.0001, 3000, 5000, 0.2, 3.0, 1.0);
    delay(1000);
    perform_one_test(0.040, 0.050, 0.0001, 3000, 5000, 0.2, 3.0, 1.0);
    delay(1000);



}

void loop() {

}
