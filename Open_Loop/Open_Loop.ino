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

/*
 *            GLOBAL OBJECTS
 */

//Motor Block
#define MOTOR_MODE 1                                                       //Motors operate in single-pin mode
AccelStepper leftMotor = AccelStepper(MOTOR_MODE, SPD_LEFT, DIR_LEFT);     //Pin 2 = Direction, Pin 3 = Speed
AccelStepper rightMotor = AccelStepper(MOTOR_MODE, SPD_RIGHT, DIR_RIGHT);  //Pin 4 = Direction, Pin 5 = Speed
 
/*
 *            GLOBAL VARIABLES
 */

float pi = 3.1416;

//Motor Block
float stepSize = 1.8;                       //Degrees
float stepSizeRad = stepSize*pi/180.;       //Radians
float stepsPerRev = 360./stepSize;          //Number of steps for one complete turn
float wheelDiam = 0.6;                      //Diameter of the driving wheel (inches)
float distPerStep = stepSizeRad*wheelDiam/2.;    //How many inches each step produces
unsigned long totalDistance;                        //Placeholder for total distance traveled

float maxSpd = 2*stepsPerRev;              //Constrain speed to 2 rev/sec = 120 rpm*maxSpd;                  
float maxAccel = .5*maxSpd;                     //Set acceleration time to 0.25 seconds

//Track block
float p1dist = 8.5;
float p2dist = p1dist + 10;
float p3dist = p2dist + 10.5;
float p4dist = p3dist + 18;

void setup() {
  Serial.begin(9600);
  Serial.println("Beginning:");
  Serial.println("Phase\tTotal Distance\tCurrent Speed\tLeft Speed\tRightSpeed\tLeftDistanceToGo\tRightDistanceToGo\tTime");

  //Motor Setup
  leftMotor.setMaxSpeed(maxSpd);
  rightMotor.setMaxSpeed(maxSpd);
  
  leftMotor.setCurrentPosition(0);
  rightMotor.setCurrentPosition(0);

  delay(2500);
}

void loop() {
  /*
   * Use phases to control behavior as cart progresses through track:
   * Phase 1-Initial straight motion
   * Phase 2-Right turn
   * Phase 3-Left turn
   * Phase 4-Straightaway with ramps
   * Phase 5-Finish (no motion)
   */
  static byte phase = 0;

  //Find total distance (in inches) to calculate which phase we're in
  //Averages distance traveled by left motor and right motor to approximate central position
  float leftPosition = float(leftMotor.currentPosition());
  float rightPosition = float(rightMotor.currentPosition());
  totalDistance = (abs(distance(leftPosition)) + abs(distance(rightPosition)))/2.;

  //Check for phase change and assign new parameters if needed
  if (totalDistance > p4dist){
    phase = 5;
    setParameters(phase);
  }
  else if (totalDistance > p3dist){
    phase = 4;
    setParameters(phase);
  }
  else if (totalDistance > p2dist){
    phase = 3;
    setParameters(phase);
  }
  else if (totalDistance > p1dist){
    phase = 2;
    setParameters(phase);
  }
  else if (phase == 0){
    phase = 1;
    setParameters(phase);
  }

  leftMotor.run();
  rightMotor.run();

  printShit(phase,totalDistance);
}

//Converts rotational distance into linear distance
float distance(float steps){
  float linear = distPerStep*steps;

  return linear;
}

//Sets the motor run parameters based on what the current phase is
void setParameters(byte state){
  switch (state){
    case 1:
      //Forward motion

      leftMotor.move(10000);
      rightMotor.move(-10000);
      
      leftMotor.setSpeed(0.5*maxSpd);
      rightMotor.setSpeed(0.5*maxSpd);

      leftMotor.setAcceleration(0.5*maxAccel);
      rightMotor.setAcceleration(0.5*maxAccel);

      //Also initialize movement in this phase

      
      break;
    case 2:
      //Turning right

      leftMotor.setSpeed(0.9*maxSpd);
      rightMotor.setSpeed(-0.15*maxSpd);
      
      break;
    case 3:
      //Turning left
      leftMotor.setSpeed(.25*maxSpd);
      rightMotor.setSpeed(-0.85*maxSpd);
      
      break;
    case 4:
      //Going forward with ramps
      leftMotor.setSpeed(maxSpd);
      rightMotor.setSpeed(-maxSpd);
      
      break;
    case 5:
      //STOP - FINISHED
      leftMotor.stop();
      rightMotor.stop();
      
      break;
    default:
      //OOPS SOMETHING BROKE
      leftMotor.stop();
      rightMotor.stop();
      
      break;
  }
}

void printShit(byte phase, int dist){
  static unsigned long refTime = 0;
  
  if (millis() - refTime > 50){
    Serial.print(phase);
    Serial.print("\t\t");
    Serial.print(dist);
    Serial.print("\t\t");
    int avgSpd = (abs(leftMotor.speed()) + abs(rightMotor.speed()))/2;
    Serial.print(avgSpd);
    Serial.print("\t");
    Serial.print(leftMotor.speed());
    Serial.print("\t\t");
    Serial.print(rightMotor.speed());
    Serial.print("\t\t");
    Serial.print(leftMotor.distanceToGo());
    Serial.print("\t\t\t");
    Serial.print(rightMotor.distanceToGo());
    Serial.print("\t\t\t");
    Serial.print(millis() - 5000);
    Serial.print("\n");
    
    refTime = millis();
  }
}
