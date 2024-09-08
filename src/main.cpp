/*
 *  Created on: 23.02.2021
 *      Author: anonymous
 * 
 *  Changelog:
 *  ### Version 1.6
 *  (20240721)
 *  - Adapted for Wii remote only, no web server
 * 
 *  ### Version 1.5
 * (20221018)
 * - Added passive buzzer support
 * - Robot waiting 45 seconds for Wifi connection. After that the robot will disable Wifi and switch to Wii-remote bluetooth
 * 
    ### Version 1.4
 *  (20220727)
 *  - Added Wii-Remote as remote controller (pressing key 1 + 2 on same time to connect the Wii Remote)
 * 
 *  ### Verison 1.3
 *  (20220703)
 *  - Added OTA firmware web update
 *  - Optimized web control
 *
 *  ### Version 1.2
 *	(20220627)
 *	- Added joystick via browser
 *	- Added background live view with esp32-came
 *	- Added light, sound, servo control
 *
 *    ### Version 1.0
 *      (20220322)
 *      - Ported to platformio
 *      - Bugfix Android app right/left direction
 * 
 *  ToDo:
 *    - Web-GUI
 *    - OTA
 * 
 * ******** Pin out **********
 * Pin 12   Stepper A&B Enable
 * Pin 26   Stepper A STEP
 * Pin 25   Stepper A DIR
 * Pin 14   Stepper B STEP
 * Pin 27   Stepper B DIR
 * 
 * Pin 13   Servo
 * 
 * Pin 21   MPU-6050 SDA
 * Pin 22   MPU-6050 SCL
 * 
 * Pin 32   LED Light
 * 
 * Pin 33   BUZZER
 * 
 * ******** Command Character *******
 * Forward              /?fader1=1.00
 * Backward             /?fader1=0.00
 * Turn Right           /?fader2=0.00
 * Turn Left            /?fader2=1.00
 * Servo Arm            /?push1=1|2
 * Beep On|Off          /?push3=1|2
 * Turn Light On|Off    /?push4=1|0
 * Mode PRO On|Off      /?toggle1=1|0
 * P-Stability          /?fader3=0.00-1.00
 * D-Stability          /?fader4=0.00-1.00
 * P-Speed              /?fader5=0.00-1.00
 * I-Speed              /?fader6=0.00-1.00
 * 
 * Controll via Wii-Remote controller
 * - Connect to robot = press 1+2 same time
 * - Button 1         = piezo
 * - Button 2         = light
 * - Home             = toggle mode pro (pro mode lights blinking)
 * - Control via gyro = press B button + gyro move to go forward/backward right/left.
 *                      hold controller crosswise
 * 
 */
 
#include <Wire.h>
#include <Arduino.h>
#include "Control.h"
#include "MPU6050.h"
#include "Motors.h"
#include "defines.h"
#include "globals.h"
#include <stdio.h>
#include "esp_types.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "esp32-hal-ledc.h"
#include "ESP32Wiimote.h"


const char* PARAM_FADER1 = "fader1";
const char* PARAM_FADER2 = "fader2";
const char* PARAM_PUSH1 = "push1";
const char* PARAM_PUSH2 = "push2";
const char* PARAM_PUSH3 = "push3";
const char* PARAM_PUSH4 = "push4";
const char* PARAM_TOGGLE1 = "toggle1";
const char* PARAM_FADER3 = "fader3";
const char* PARAM_FADER4 = "fader4";
const char* PARAM_FADER5 = "fader5";
const char* PARAM_FADER6 = "fader6";


// Struct for tasks
struct task
{
    unsigned long rate;
    unsigned long previous;
};

task taskA = {.rate = 397, .previous = 0};      // 397 ms
task taskB = {.rate = 997, .previous = 0};      // 997 ms

/* Wii-Remote*/
ESP32Wiimote wiimote;
static bool logging = true; //MPW_ENG debugging on
static long last_ms = 0;


// Firmware version
String FW_VERSION = "2.0";

unsigned long previousMillis = 0;


// Gyro routine
void initMPU6050() {

  MPU6050_setup();
  delay(500);
  MPU6050_calibrate();
  
}

void initTimers();


void setup() {
  Serial.begin(115200);         // set up seriamonitor at 115200 bps
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("*mpweng Balancing Robot*");
  Serial.println("--------------------------------------------------------");

  pinMode(PIN_ENABLE_MOTORS, OUTPUT);
  digitalWrite(PIN_ENABLE_MOTORS, HIGH);
  
  pinMode(PIN_MOTOR1_DIR, OUTPUT);
  pinMode(PIN_MOTOR1_STEP, OUTPUT);
  pinMode(PIN_MOTOR2_DIR, OUTPUT);
  pinMode(PIN_MOTOR2_STEP, OUTPUT);
  pinMode(PIN_SERVO, OUTPUT);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  pinMode(PIN_WIFI_LED, OUTPUT);
  digitalWrite(PIN_WIFI_LED, LOW);
  
  // Pip buzzer 
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, HIGH);
  delay(100);
  digitalWrite(PIN_BUZZER, LOW);

  // Servo
  ledcSetup(6, 50, 16); // channel 6, 50 Hz, 16-bit width
  ledcAttachPin(PIN_SERVO, 6);   // GPIO 22 assigned to channel 6
  delay(50);
  ledcWrite(6, SERVO_AUX_NEUTRO);

  Wire.begin();
  initMPU6050();

  /* Wii-Remote init */
  wiimote.init();
  // wiimote.addFilter(ACTION_IGNORE, FILTER_ACCEL); // optional - uncomment this will filter all acclerations!
  Serial.println("Wii-init done!");
  last_ms = millis();
  
  // default neutral values
  OSCfader[0] = 0.5;
  OSCfader[1] = 0.5;
  OSCfader[2] = 0.5;
  OSCfader[3] = 0.5;

  initTimers();

  //digitalWrite(PIN_ENABLE_MOTORS, LOW);
  for (uint8_t k = 0; k < 5; k++) {
    setMotorSpeedM1(5);
    setMotorSpeedM2(5);
    ledcWrite(6, SERVO_AUX_NEUTRO + 250);
    delay(200);
    setMotorSpeedM1(-5);
    setMotorSpeedM2(-5);
    ledcWrite(6, SERVO_AUX_NEUTRO - 250);
    delay(200);
  }
  ledcWrite(6, SERVO_AUX_NEUTRO);
}

void processOSCMsg() {
  if (OSCpage == 1) {
    if (modifing_control_parameters)  // We came from the settings screen
    {
      OSCfader[0] = 0.5; // default neutral values
      OSCfader[1] = 0.5; // default neutral values
      OSCtoggle[0] = 0;  // Normal mode
      mode = 0;
      modifing_control_parameters = false;
    }

    if (OSCmove_mode) {
      positionControlMode = true;
      OSCmove_mode = false;
      target_steps1 = steps1 + OSCmove_steps1;
      target_steps2 = steps2 + OSCmove_steps2;
    } else {
      positionControlMode = false;
      throttle = (OSCfader[0] - 0.5) * max_throttle;
      // We add some exponential on steering to smooth the center band
      steering = OSCfader[1] - 0.5;
      if (steering > 0)
        steering = (steering * steering + 0.5 * steering) * max_steering;    
      else
        steering = (-steering * steering + 0.5 * steering) * max_steering;    
    }

    if ((mode == 0) && (OSCtoggle[0])) {
      // Change to PRO mode
      max_throttle = MAX_THROTTLE_PRO;
      max_steering = MAX_STEERING_PRO;
      max_target_angle = MAX_TARGET_ANGLE_PRO;
      mode = 1;
    }
    if ((mode == 1) && (OSCtoggle[0] == 0)) {
      // Change to NORMAL mode
      max_throttle = MAX_THROTTLE;
      max_steering = MAX_STEERING;
      max_target_angle = MAX_TARGET_ANGLE;
      mode = 0;
    }
  } else if (OSCpage == 2) { // OSC page 2
    if (!modifing_control_parameters) {
      for (uint8_t i = 0; i < 4; i++)
        OSCfader[i] = 0.5;
      OSCtoggle[0] = 0;
      modifing_control_parameters = true;
    }
    // MPW_ENG - Disable PID Tuning
    // User could adjust KP, KD, KP_THROTTLE and KI_THROTTLE (fadder3,4,5,6)
    // Now we need to adjust all the parameters all the times because we dont know what parameter has been moved
    // Kp_user = KP * 2 * OSCfader[0];
    // Kd_user = KD * 2 * OSCfader[1];
    // Kp_thr_user = KP_THROTTLE * 2 * OSCfader[2];
    // Ki_thr_user = KI_THROTTLE * 2 * OSCfader[3];
    // // Send a special telemetry message with the new parameters
    // char auxS[50];
    // sprintf(auxS, "$tP,%d,%d,%d,%d", int(Kp_user * 1000), int(Kd_user * 1000), int(Kp_thr_user * 1000), int(Ki_thr_user * 1000));



    // Calibration mode??
    if (OSCpush[2] == 1) {
      Serial.print("Calibration MODE ");
      angle_offset = angle_adjusted_filtered;
      Serial.println(angle_offset);
    }

    // Kill robot => Sleep
    while (OSCtoggle[0] == 1) {
      //Reset external parameters
      PID_errorSum = 0;
      timer_old = millis();
      setMotorSpeedM1(0);
      setMotorSpeedM2(0);
      digitalWrite(PIN_ENABLE_MOTORS, HIGH);  // Disable motors
    }
  }
}


void loop() {

  static bool bKill = false;
  static bool bWiiActive = false;
  static bool wiimote_cal = false;
  static bool bCalTmUp = false;
  static float fAvAngle = 0;

  //Wii Remote accelerometer linear gain and offset
  static float xOffset = -1.836;
  static float yOffset = 2.836;
  static float xGain = 0.0182;
  static float yGain = -0.0182;

  //task A
  if (taskA.previous == 0 || (millis() - taskA.previous > taskA.rate)) {
    taskA.previous = millis(); 
    // 397 ms
    action = true;  // debounce actions

    // Return Servo to neutral
    OSCpush[0]=0;
    ledcWrite(6, SERVO_AUX_NEUTRO);

    // blinking eyes during PRO MODE
    if (OSCtoggle[0] == 1) {
        digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    }

    //Reset buzzer
    if (!wiimote_cal)
    digitalWrite(PIN_BUZZER, LOW);

    //Flash WiFi LED (Wii Remoite Mode)
    if (bWiiActive)
      digitalWrite (PIN_WIFI_LED, !digitalRead(PIN_WIFI_LED));
  }

  //task B
  if (taskB.previous == 0 || (millis() - taskB.previous > taskB.rate)) {
    taskB.previous = millis(); 
    // 997 ms

    if (wiimote_cal) {
      wiimote_cal = false;
      bCalTmUp = true;
    }

    if (logging){    
      Serial.print(F("Throttle: "));
      Serial.println(OSCfader[0]);   
      Serial.print(F("Steering: "));
      Serial.println(OSCfader[1]);
      Serial.print(F("Angle: "));
      Serial.println(angle_adjusted);
      Serial.print(F("dt: "));
      Serial.println(dt*1000);
    }   
  }


  if (true) {
    /* Wii-Remote */
    wiimote.task();
  

    if (wiimote.available() > 0) {
      ButtonState  button  = wiimote.getButtonState();
      AccelState   accel   = wiimote.getAccelState();
      //NunchukState nunchuk = wiimote.getNunchukState();

      /* process buttons from Wii */
      
      // Button A activates servo if wii remote active 
      // and triggers calibrates remote if not
      if (button & BUTTON_A) {
        if (action) { 
          taskA.previous = millis();
          action = false;
           OSCpush[0]=1;
        }
      }

      if (button & BUTTON_UP) {
        if (!bWiiActive){
          taskB.previous = millis();
          digitalWrite(PIN_BUZZER, HIGH);
          wiimote_cal = true;
        }
      }

      //after task B timer up, cal wii remote
      if (bCalTmUp){
        bCalTmUp = false;
        xOffset = 0.5 - xGain*accel.xAxis;
        yOffset = 0.5 - yGain*accel.yAxis;
        digitalWrite(PIN_BUZZER, LOW);
      }

      // Buzzer button (pulse)
      if (button & BUTTON_ONE) {
        if (action) { 
          taskA.previous = millis();
          action = false;
          digitalWrite(PIN_BUZZER, HIGH);
        }
      }
      // LED eyes (toggle)
      if (button & BUTTON_TWO) {
          // toggle need
        if (action) { 
          taskA.previous = millis(); 
          action = false;
          digitalWrite(PIN_LED, !digitalRead(PIN_LED));
        }
      }
      // Switch ProMode (toggle)
      if (button & BUTTON_HOME) {     
        if (action) { 
          taskA.previous = millis(); 
          action = false;
          OSCnewMessage = 1;
          OSCpage = 1;
          if (OSCtoggle[0] == 1) {
              OSCtoggle[0] = 0;
          } else {
              OSCtoggle[0] = 1;
          }
        }
      }
      // Kill Robot (toggle)
      if (button & BUTTON_MINUS) {
          // toggle need
        if (action) { 
          taskA.previous = millis(); 
          action = false;
          bKill = !bKill;
        }
      }
      // Wii Remote Controls Active (toggle)
      if (button & BUTTON_PLUS) {
          // toggle need
        if (action) { 
          taskA.previous = millis(); 
          action = false;
          bWiiActive = !bWiiActive;
        }
      }
      // Calculate Forward/Backward Left/Right
      if (bWiiActive) {
        OSCfader[0] = xGain*accel.xAxis + xOffset;
        OSCfader[1] = yGain*accel.yAxis + yOffset; 
        OSCnewMessage = 1;
        OSCpage = 1; 
      } else if (OSCfader[0] != 0.5 || OSCfader[1] != 0.5) {
        OSCfader[0] = 0.5;
        OSCfader[1] = 0.5;
        OSCnewMessage = 1;
        OSCpage = 1; 
      }

      /* Wii-Remote END */
    }
  }

  if (OSCnewMessage) {
    OSCnewMessage = 0;
    processOSCMsg();
  }

  timer_value = micros();

  if (MPU6050_newData()) {
    
    MPU6050_read_3axis();
    
    dt = (timer_value - timer_old) * 0.000001; // dt in seconds
    timer_old = timer_value;

    angle_adjusted_Old = angle_adjusted;
    // Get new orientation angle from IMU (MPU6050)
    float MPU_sensor_angle = MPU6050_getAngle(dt);

    angle_adjusted = MPU_sensor_angle;
    if ((MPU_sensor_angle > -15) && (MPU_sensor_angle < 15)){
      angle_adjusted_filtered = angle_adjusted_filtered * 0.5 + MPU_sensor_angle * 0.5; // MPW_ENG (Old 0.99, 0.01)
    }

    // We calculate the estimated robot speed:
    // Estimated_Speed = angular_velocity_of_stepper_motors(combined) - angular_velocity_of_robot(angle measured by IMU)
    actual_robot_speed = (speed_M1 + speed_M2) / 2; // Positive: forward

    int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 25.0; // 25 is an empirical extracted factor to adjust for real units
    int16_t estimated_speed = -actual_robot_speed + angular_velocity;

    estimated_speed_filtered = estimated_speed_filtered * 0.9 + (float) estimated_speed * 0.1;

    //MPW_ENG - remove position control mode
    // if (positionControlMode) {
    //   // POSITION CONTROL. INPUT: Target steps for each motor. Output: motors speed
    //   motor1_control = positionPDControl(steps1, target_steps1, Kp_position, Kd_position, speed_M1);
    //   motor2_control = positionPDControl(steps2, target_steps2, Kp_position, Kd_position, speed_M2);

    //   // Convert from motor position control to throttle / steering commands
    //   throttle = (motor1_control + motor2_control) / 2;
    //   throttle = constrain(throttle, -190, 190);
    //   steering = motor2_control - motor1_control;
    //   steering = constrain(steering, -50, 50);
    // }

    // ROBOT SPEED CONTROL: This is a PI controller.
    //    input:user throttle(robot speed), variable: estimated robot speed, output: target robot angle to get the desired speed
    target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr);
    target_angle = constrain(target_angle, -max_target_angle, max_target_angle); // limited output

    // Stability control (100Hz loop): This is a PD controller.
    //    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
    //    We integrate the output (sumatory), so the output is really the motor acceleration, not motor speed.
    control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
    control_output = constrain(control_output, -MAX_CONTROL_OUTPUT,  MAX_CONTROL_OUTPUT); // Limit max output from control

    // The steering part from the user is injected directly to the output
    motor1 = control_output + steering;
    motor2 = control_output - steering;

    // Limit max speed (control output)
    motor1 = constrain(motor1, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
    motor2 = constrain(motor2, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

    int angle_ready;
    if (OSCpush[0]){     // If we press the SERVO button we start to move
      angle_ready = 90;  // MPW_ENG (Old 82)
      if (angle_adjusted > -40) 
        ledcWrite(6, SERVO_AUX_NEUTRO + 3500);
      else 
        ledcWrite(6, SERVO_AUX_NEUTRO - 3000);
    } 
    else {
      angle_ready = 70;  // Default angle MPW_ENG (old 74)
      
    }

    if ((angle_adjusted < angle_ready) && (angle_adjusted > -angle_ready) && (bKill==false)) // Is robot ready (upright?)
      {
      // NORMAL MODE
      digitalWrite(PIN_ENABLE_MOTORS, LOW);  // Motors enable
      // NOW we send the commands to the motors
      setMotorSpeedM1(motor1);
      setMotorSpeedM2(motor2);
    } else   // Robot not ready (flat), angle > angle_ready => ROBOT OFF
    {
      digitalWrite(PIN_ENABLE_MOTORS, HIGH);  // Disable motors
      setMotorSpeedM1(0);
      setMotorSpeedM2(0);
      PID_errorSum = 0;  // Reset PID I term
      Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;
      // RESET steps
      steps1 = 0;
      steps2 = 0;
      positionControlMode = false;
      OSCmove_mode = false;
      throttle = 0;
      steering = 0;
    }
    
    // Normal condition?
    if ((angle_adjusted < 56) && (angle_adjusted > -56)) {
      Kp = Kp_user;            // Default user control gains
      Kd = Kd_user;
      Kp_thr = Kp_thr_user;
      Ki_thr = Ki_thr_user;
    } else // We are in the raise up procedure => we use special control parameters
    {
      Kp = KP_RAISEUP;         // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;
      PID_errorSum = 0;
    }

  } // End of new IMU data

  

}



