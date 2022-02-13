#include <ros.h>
#include <std_msgs/String.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include "Encoder_CFG.h"
#include "odometry.h"
#include "pin_assign.h"
#include <Wire.h>
#include <VL53L0X.h> // TOF Sensors library


#define XSHUT_pin3 PB1 //shut down pin for first sensor
#define XSHUT_pin2 PA15 //shut down pin for second sensor
#define XSHUT_pin1 PB0 //shut down pin for third sensor
#define LED_ON  HIGH
#define LED_OFF LOW

#define Sensor1_newAddress 43
#define Sensor2_newAddress 45
#define Sensor3_newAddress 46

static bool ledState;

ros::NodeHandle  nh;

#define ODO_PERIOD 200  // Millis between /tf and /odom publication
#define PID_PERIOD  50  // Millis between each PID calculation


Encoder_CFG encoderLeft( ENC1A, ENC1B);
Encoder_CFG encoderRight(ENC2A, ENC2B);


// The followont two funtions look pointless but are required by the Arduino
// system because ISPs appearenly can not be non-static members of a class.
void ispLeft() {
  encoderLeft.tick();
}
void ispRight() {
  encoderRight.tick();
}

// TODO:  meterPerTick needs to be computed from parameters in setup()
// meter per encoder tick is wheel circumfrence / encoder ticks per wheel revoution
//const float meterPerTick = (0.13 * 3.1415) / (75.0 * 64.0); // Thumper

const float meterPerTick = 0.00174603174;  // needs review
const float base_width   = 0.62;         // Base width for AGV


long encoderLeftLastValue  = 0L;
long encoderRightLastValue = 0L;

Odometer odo(meterPerTick, base_width);



#include "Cytron_setup.h"

MotoCytron moto;
#define LEFT_MOTOR   0
#define RIGHT_MOTOR  1

#include <PID_v1.h>

// These global variables are used by the PID library.
// TODO Kp, Ki and Kd should be parameters
double leftSetpoint = 0.0;
double leftInput,  leftOutput;
double rightSetpoint = 0.0;
double rightInput, rightOutput;

//double Kp = 60, Ki = 100, Kd = 1;  //  Usable but not respncive enough
double Kp = 40.0, Ki = 0.0, Kd = 0.0;

// Create one PID object for each motor.  The Input and output units
// will be "meters"
PID leftWheelPID( &leftInput,  &leftOutput,  &leftSetpoint,  Kp, Ki, Kd, DIRECT);
PID rightWheelPID(&rightInput, &rightOutput, &rightSetpoint, Kp, Ki, Kd, DIRECT);



#include <LiquidCrystal.h>
const int rs = PA0, en = PA1, d4 = PA2, d5 = PA3, d6 = PA4, d7 = PA5;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
//#endif // HAVE_LCD

// Functions declaratons (silly C-language requirement)
void cmd_velCallback( const geometry_msgs::Twist& toggle_msg);
void broadcastTf();
// DEBUG FOLLOWS
void MotorTest();

// Subscribe to Twist messages on cmd_vel.

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", &cmd_velCallback);

// This is an Arduino convention.  Place everything that needs to run just
// TOF sensors addressing
VL53L0X Sensor1;
VL53L0X Sensor2;
VL53L0X Sensor3;

// once in the setup() funtion.  The environment will call setup()


void setup() {
      //lcd intialization
    lcd.begin(16, 2);
// TOF pin mode
  pinMode(XSHUT_pin1, OUTPUT);
  pinMode(XSHUT_pin2, OUTPUT);
  pinMode(XSHUT_pin3, OUTPUT);
  
  Wire.begin(); //I2C intialization
  
  Sensor1.setAddress(Sensor1_newAddress);
  pinMode(XSHUT_pin1, INPUT);
  delay(10);
  
  Sensor2.setAddress(Sensor2_newAddress);
  pinMode(XSHUT_pin2, INPUT);
  delay(10);

  Sensor3.setAddress(Sensor3_newAddress);
  pinMode(XSHUT_pin3, INPUT);
  delay(10); //For power-up procedure t-boot max 1.2ms "Datasheet: 2.9 Power sequence"

    // TOF sensors intialization & Timout
  Sensor1.init();
  Sensor2.init();
  Sensor3.init();
  Sensor1.setTimeout(500);
  Sensor2.setTimeout(500);
  Sensor3.setTimeout(500);

  
  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  Sensor1.startContinuous(100);
  Sensor2.startContinuous(100);
  Sensor3.startContinuous(100);

  moto.setup();

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(PC13, OUTPUT);

  // Make sure the motors are stopped.
  // Call the driver directly and also set the PID setpoints.
  moto.motorOff(LEFT_MOTOR);
  moto.motorOff(RIGHT_MOTOR);
  leftSetpoint  = 0.0;
  rightSetpoint = 0.0;


  // Set up interrupt on encoder pins.   NOte the function ispLeft and ispRight
  // are required by the Arduino sysem because ISPs can not be non-static class
  // members.
  encoderLeft.setup();
  encoderRight.setup();
  attachInterrupt(digitalPinToInterrupt(ENC1A), ispLeft,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1B), ispLeft,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2A), ispRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2B), ispRight, CHANGE);

  // Start PID controlers. All we need next is data
  leftWheelPID.SetSampleTime(PID_PERIOD);
  rightWheelPID.SetSampleTime(PID_PERIOD);
  leftWheelPID.SetOutputLimits(-225, 225);
  rightWheelPID.SetOutputLimits(-255, 255);
  leftWheelPID.SetMode(AUTOMATIC);
  rightWheelPID.SetMode(AUTOMATIC);

  // Connect to ROS computer and wait for connection
  nh.initNode();

  // Advertize odometry and transform
  odo.setupPubs(nh);

  // Subscribe to cmd_vel
  nh.subscribe(sub_cmd_vel);

  nh.loginfo("starting...");
  lcd.setCursor(0,0);
  lcd.print("AMR ROBOT");
  delay(500);
  lcd.setCursor(0,1);
  lcd.print("Communicating...");
  delay(1000);
  lcd.clear();
}


// These are all used in the loop() below
static long encoderLeftLastValuePid  = 0;
static unsigned long millis_last_LEFT = 0;

static long encoderRightLastValuePid  = 0;
static unsigned long millis_last_RIGHT = 0;

static unsigned long NextPubMillis   = 0;
static long encoderLeftLastValueOdo  = 0;
static long encoderRightLastValueOdo = 0;
static long timeLastOdo              = 0;

// This loop() funtion is an arduino convention.  It is called by the environment
// inside a tight loop and runs forever or until the CPU is reset or powered off.
//
void loop() {
  float seconds_from_last;
  long  millis_from_last;
  float distLeft;
  float distRight;

  
  // Three things run here all on their own schedule
  //  1.  The Left Wheel PID controler
  //  2.  The Right Wheel PID controler
  //  3.  The Odometry and TF publisher
  // Each wheel has it's own PID control and might do it's computation at
  // different times.
    
  // Get encoder values
  long encLeft  = encoderLeft.getPos();
  long encRight = encoderRight.getPos();
  long curMillis = millis();  // capture time when encoders are sampled


  //=========> LEFT PID Controler
  //
  // Figure out how far we have gone in meters from last PID computation
  distLeft  = meterPerTick * float(encLeft  - encoderLeftLastValuePid);
  
  //figure out how fast the LEFT wheel went, in meters per second
  millis_from_last  = curMillis - millis_last_LEFT;
  seconds_from_last = float(millis_from_last) / 1000.0;
  leftInput  = distLeft  / seconds_from_last;

  // The PID.Compute() method will look at the millis() clock and determine if it is
  // time to calculate new output.   If so it returns true and then we update the
  // motor speed.  Note the motor update speed update rate is independent of the /tf
  // and /odom publication rate.
  if (leftWheelPID.Compute()) {
    setMotorSpeed(LEFT_MOTOR, leftOutput);
    
    encoderLeftLastValuePid = encLeft;
    millis_last_LEFT = curMillis;
  }
  
  //==========> RIGHT PID Controler
  //
  // Figure out how far we have gone in meters from last PID computation
  distRight = meterPerTick * float(encRight - encoderRightLastValuePid);
  
  //figure out how fast the RIGHT wheel went, in meters per second
  millis_from_last  = curMillis - millis_last_RIGHT;
  seconds_from_last = float(millis_from_last) / 1000.0;
  rightInput  = distRight  / seconds_from_last;

  // The PID.Compute() method will look at the millis() clock and determine if it is
  // time to calculate new output.   If so it returns true and then we update the
  // motor speed.  Note the motor update speed update rate is independent of the /tf
  // and /odom publication rate.
  if (rightWheelPID.Compute()) {
    setMotorSpeed(RIGHT_MOTOR, rightOutput);
    
    encoderRightLastValuePid = encRight;
    millis_last_RIGHT = curMillis;
  }

  //==========> OdometryPublsher
  //
  // Check if it is time to publish /odom and /tf
  if (curMillis >= NextPubMillis) {
    NextPubMillis = curMillis + ODO_PERIOD;

    // Figure out how far we have gone in meters from last PID computation
    distLeft  = meterPerTick * float(encLeft  - encoderLeftLastValueOdo);
    distRight = meterPerTick * float(encRight - encoderRightLastValueOdo);

  
    // Blink the LED to show we are alive
    toggleLED();
  
    // Publish odometry
    float odoInterval = float(curMillis - timeLastOdo) / 1000.0;
    odo.update_publish(nh.now(), odoInterval, distLeft, distRight);

    encoderLeftLastValueOdo  = encLeft;
    encoderRightLastValueOdo = encRight;
    timeLastOdo = curMillis;
    }
  int dist1=Sensor1.readRangeContinuousMillimeters() ;
  int dist2= Sensor2.readRangeContinuousMillimeters();
  int dist3=Sensor3.readRangeContinuousMillimeters();

  nh.spinOnce();
  delay(1);
}


// This funtion is called every time we receive a Twist message.
// We do not send the commanded speed to the wheels.  Rather we set
// thePID loops set point to the commanded speed.
void cmd_velCallback( const geometry_msgs::Twist& twist_msg) {

  

  // We only use two numbers from the Twist message.
  //    linear.x  is the forward speed in meters per second.
  //              (The "x" axis points forward.)
  //    angular.y is the rotation about the z or vertical
  //              axis in radians per second.
  //
  float vel_x   = twist_msg.linear.x;
  float vel_th  = twist_msg.angular.z;

  // This is a "hack".  It turns ou the motors have a minimum
  // speed because of internal friction.   If the commanded speed is
  // below a threshold we replace the commanded speed with zero.
  // TODO:  Find a better threshold, make it a parameter
  if (fabs(vel_x)  < 0.001) vel_x  = 0.0;
  if (fabs(vel_th) < 0.001) vel_th = 0.0;

  // Compute the wheel speeds in meters per second.
  float left_vel  =  vel_x - (vel_th * base_width / 2.0);
  float right_vel =  vel_x + (vel_th * base_width / 2.0);

  char buff[40];
  snprintf(buff, 100, "CMD_VEL %f, %f", left_vel, right_vel);
  nh.loginfo(buff);

  // Show the Twist message on the LCD.
  displayStatus(&vel_x, &vel_th);
  
  // The PID works in units of meters per second, so no
  // conversion is needed.
  leftSetpoint  = left_vel;
  rightSetpoint = right_vel;
}

void setMotorSpeed(byte motor, float pidOutput) {
  // Set the controler based on calulation from PID
  const float deadZone = 0.5;
  byte direction;

  int speed  = int(0.5 + fabs(pidOutput));
  
  if (pidOutput >  deadZone) {
    direction = CW;
  }
  else if (pidOutput < -deadZone) {
    direction = CCW;
  }
  else {
    direction = BRAKEGND;
    speed = 0;
  }
  moto.motorGo(motor, direction, speed);
}

void displayStatus(float *vel_x, float *vel_th) {
  // set the cursor (column, line)
  lcd.setCursor(0, 0);
  lcd.print("X");
  lcd.print(*vel_x);

  lcd.setCursor(0, 1);
  lcd.print("T");
  lcd.print(*vel_th);
}

void toggleLED() {
  if (ledState) {
    digitalWrite(PC13, LED_ON);
  }
  else {
    digitalWrite(PC13, LED_OFF);
  }
  ledState = !ledState;
}
