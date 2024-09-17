#include <AccelStepper.h>
#include <MultiStepper.h>
#include <NewPing.h>

/*
 *            DEFINITIONS
 */

//US Pins
#define TRIG_LEFT 8
#define ECHO_LEFT 9
#define TRIG_RIGHT 10
#define ECHO_RIGHT 11

//Motor Pins
#define DIR_LEFT 4
#define SPD_LEFT 5
#define DIR_RIGHT 2
#define SPD_RIGHT 3

//LED Pins
#define LED_PIN 13

//BOOL Pins
#define TILT_IN 51
#define TILT_OUT 53
/*
 *            GLOBAL OBJECTS
 */

//Motor Block
#define MOTOR_MODE 1                                                       //Motors operate in single-pin mode
AccelStepper leftMotor = AccelStepper(MOTOR_MODE, SPD_LEFT, DIR_LEFT);     //Pin 2 = Direction, Pin 3 = Speed
AccelStepper rightMotor = AccelStepper(MOTOR_MODE, SPD_RIGHT, DIR_RIGHT);  //Pin 4 = Direction, Pin 5 = Speed

//Ultrasound Block
#define MAX_DISTANCE 50                                                    //
NewPing leftUS(TRIG_LEFT, ECHO_LEFT, MAX_DISTANCE);
NewPing rightUS(TRIG_RIGHT, ECHO_RIGHT, MAX_DISTANCE);

/*
 *            GLOBAL VARIABLES
 */

//General
float dt;
unsigned long curTime = 0;
float pi = 3.1416;

//Motor Block
float stepSize = 1.8;                       //Degrees
float stepSizeRad = stepSize*pi/180.;       //Radians
float stepsPerRev = 360./stepSize;          //Number of steps for one complete turn
float wheelDiam = 0.6;                      //Diameter of the driving wheel (inches)
float distPerStep = stepSizeRad*wheelDiam/2.; 

float maxSpd = 4*stepsPerRev;               //Constrain speed to 2 rev/sec = 120 rpm
float avgSpd = maxSpd/2.;                   //Set average speed for each wheel at half of the maximum speed (guarantees forward movement)
float maxAccel = 4*avgSpd;                  //Set acceleration time to even out speed adjustments

//Ultrasound Sensor block
float leftDist;
float rightDist;

//PID Block
float KU = 100;                              //Ultimate gain (estimated)
float TU = 10;                               //Oscillation period in seconds (estimated)

//PID parameters (Ziegler-Nichols Method)
float kp = 0.6*KU;                          //Proportionality constant
//float ki = 1.2*KU/TU;                       //Integral constant
float ki = 0;

float proportional;
float integral = 0;

unsigned long totalDistance;

void setup() {
  Serial.begin(9600);
  Serial.println("Left Dist\t\tRight Dist\t\tLeft Speed\t\tRight Speed\tP\tI");

  pinMode(TILT_IN, INPUT);
  pinMode(TILT_OUT, OUTPUT);
  digitalWrite(TILT_OUT, HIGH);

  //Motor setup
  leftMotor.setCurrentPosition(0);
  rightMotor.setCurrentPosition(0);
  
  leftMotor.setMaxSpeed(maxSpd);
  rightMotor.setMaxSpeed(maxSpd);
  leftMotor.setAcceleration(maxAccel);
  rightMotor.setAcceleration(maxAccel);

  delay(5000);                              //Hold for 5s on startup
}

void loop() {
  float speedMod;
  static unsigned long USTimer = 0;
  
  int dtUS = millis() - USTimer;
  dt = (millis() - curTime)/1000.;          //Delta-time in seconds
  
  float leftPosition = float(leftMotor.currentPosition());
  float rightPosition = float(rightMotor.currentPosition());
  totalDistance = (abs(distance(leftPosition)) + abs(distance(rightPosition)))/2.;

  leftDist = leftUS.ping_cm();              //Left and right distances in centimeters
  rightDist = rightUS.ping_cm();
  if (leftDist > 30 || leftDist == 0) {
    leftDist = 30;
  }
  if (rightDist > 30 || rightDist == 0) {
    rightDist = 30;
  }

  if (dtUS > 100) {
    //Check for end conditions (both sensors past wall)
    if (leftDist > 25 && rightDist > 25 && totalDistance > 50) {
      leftMotor.stop();
      rightMotor.stop();
      ledON();
      delay(10000);
    }
    //Otherwise continue running the drive loop
    else {
      //Find the output turn rate from the PID controller and evenly distribute it between both motors.
      speedMod = PIDCalc();                  
      leftMotor.move(500);
      rightMotor.move(-500);
      leftMotor.setSpeed(avgSpd*(1. - speedMod));
      rightMotor.setSpeed(-avgSpd*(1. + speedMod));
      ledBlink(1000);
      printer();
      curTime = millis();
    }
    USTimer = millis();
  }
  
  leftMotor.run();
  rightMotor.run();
}

float PIDCalc() {
  float newError;
  static float oldError = 0;
  float num;
  float output;

  newError = (leftDist - rightDist);
  proportional = newError;
  integral = integral + (newError + oldError)/2.*dt;
  
  num = kp*proportional + ki*integral;
  num = constrain(num,-avgSpd,avgSpd);

  output = float(num)/float(avgSpd);
  oldError = newError;
  
  return output;
}

void ledBlink(int timer) {
  static unsigned long blinkTime = 0;

  long delta = millis() - blinkTime;

  if (delta > timer){
    if (digitalRead(LED_PIN)) ledOFF();
    else ledON();

    blinkTime = millis();
  }
}

void ledON() {
  digitalWrite(LED_PIN,HIGH);
}

void ledOFF() {
  digitalWrite(LED_PIN,LOW);
}

void printer() {
  Serial.print(leftDist);
  Serial.print("\t\t\t");
  Serial.print(rightDist);
  Serial.print("\t\t\t");
  Serial.print(leftMotor.speed());
  Serial.print("\t\t\t");
  Serial.print(rightMotor.speed());
  Serial.print("\t\t");
  Serial.print(proportional);
  Serial.print("\t");
  Serial.print(integral);
  Serial.print("\n");
}

float distance(float steps){
  float linear = distPerStep*steps;

  return linear;
}
