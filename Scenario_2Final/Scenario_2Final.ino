#include <Wire.h>
#include <Zumo32U4.h>
#include <time.h>

#define LEFT 0
#define RIGHT 1
#define QTR_THRESHOLD 1
#define NUM_SENSORS 3
unsigned int lineSensorValues[NUM_SENSORS];
const uint16_t maxSpeed = 100;  
const uint16_t turnSpeedMax = 100;
const uint16_t turnSpeedMin = 100;
const uint16_t reverseSpeedMax = -100;
const uint8_t sensorThreshold = 10;
const uint8_t proxThreshold = 6;

char Path[100];
uint8_t pathLength = 0;

time_t start = time(0);
time_t Times[100];
int timesLength = 0;

//Addition for Scenario 2
int housesToFind = 3;
int housesFound;

int counter = 0;
bool turningRight = false;
bool turningLeft = false;
bool houseFound = false;

bool direction;
char returnDirection;
time_t returnTime;

Zumo32U4LCD display;
Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4ButtonA buttonA;


#include "TurnSensor.h"
#include "GridMovement.h"

int16_t lastError = 0;

void calibrateSensors()
{
  display.clear();

  // Wait 1 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(1000);
  for(uint16_t i = 0; i < 120; i++)
  {
    if (i > 30 && i <= 90)
    {
      motors.setSpeeds(-200, 200);
    }
    else
    {
      motors.setSpeeds(200, -200);
    }

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);

}

void setup()
{
  lineSensors.initThreeSensors();

  // Play a little welcome song
  buzzer.play(">g32>>c32");


  buttonA.waitForButton();

  calibrateSensors();
}

void forward()
{
  motors.setSpeeds(100, 100);
  Path[pathLength] = 'S';
  pathLength++;
  Times[timesLength] = difftime( time(0), start);
  timesLength++;
}

void turnAround()
{
  motors.setSpeeds(100, -100);
  Path[pathLength] = 'B';
  pathLength++;  
  Times[timesLength] = difftime( time(0), start);
  timesLength++;
  delay(2500);
}

void turnAroundComeHome()
{
  motors.setSpeeds(100, -100);
  delay(2500);
}

void turnRight()
{
  motors.setSpeeds(100, -100);
  delay(1250);
  stop();
  Path[pathLength] = 'R';
  pathLength++;  
  Times[timesLength] = difftime( time(0), start);
  timesLength++;
}

void turnLeft()
{
  motors.setSpeeds(-100, 100);
  delay(1250);
  stop();  
  Path[pathLength] = 'L';
  pathLength++;
  Times[timesLength] = difftime( time(0), start);
  timesLength++;
}

void reverse()
{
  motors.setSpeeds(-100, -100);  
  Path[pathLength] = 'B';
  pathLength++;
  Times[timesLength] = difftime( time(0), start);
  timesLength++;
}

void stop()
{
  motors.setSpeeds(0, 0);
}

//Method used to detect object using the proximity sensors
//Used to detect the presence of a house
bool proximityObject()
{
  proxSensors.initThreeSensors();
  proxSensors.read();

  uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
  uint8_t rightValue = proxSensors.countsFrontWithRightLeds();

  return leftValue >= proxThreshold || rightValue >= proxThreshold;
}

bool isLineDetected(int index)
{
  return lineSensorValues[index] >= sensorThreshold;
}

bool isLineAhead()
{ 
  return isLineDetected(1);
}

bool isDeadEnd()
{
  return isLineDetected(0) && isLineDetected(1) && isLineDetected(2);
}

bool isIntersection()
{
  return !isLineDetected(0) || !isLineDetected(2);
}

bool isStraight()
{
  return !isLineDetected(1);
}

//Method used when at an intersection to dectect where the robot can turn
//without crossing a line
void findIntersectionExits(bool * foundLeft, bool * foundStraight, bool * foundRight)
{
  *foundLeft = 0;
  *foundStraight = 0;
  *foundRight = 0;
    ledRed(1);

    lineSensors.readLine(lineSensorValuesGM, QTR_EMITTERS_ON, 1);
    if(!isLineDetected(0))
    {
      *foundLeft = 1;
    }
    if(!isLineDetected(2))
    {
      *foundRight = 1;
    } 

  lineSensors.readLine(lineSensorValuesGM, QTR_EMITTERS_ON, 1);
  if(!isLineDetected(1))
  {
    *foundStraight = 1;
  }
  ledRed(0);
}


//Main Method used to solve the maze
void leftHandMazeSolve()
{  
  bool foundLeftMS, foundStraightMS, foundRightMS = 0;  
    ledGreen(0);
    ledRed(0);    
    ledYellow(0);

    forward();
    delay(100);   

    lineSensors.readLine(lineSensorValuesGM, QTR_EMITTERS_ON, 1);

      if(proximityObject())
      {
        stop();
        delay(500);
        houseFound = true;
        housesFound++;
        return;
      }      

      if(isLineAhead())
      {
      // There line visible ahead, and we didn't see any
      // intersection.  Must be a dead end.
        ledRed(1);
        stop();
        delay(100);
        turnAround();
        delay(200);
        return;
      }      
      else if(isIntersection())
      {
      // Found an intersection
        stop(); 
        ledGreen(1);

        findIntersectionExits(&foundLeftMS, &foundStraightMS, &foundRightMS);  
        delay(100);

        if(foundLeftMS)
        {
          turnLeft();
          return;
        }
        else if(foundStraightMS)
        {
          forward();
          return;
        }
        else if(foundRightMS)
        {
          turnRight();   
          return;
        }
      }


  //buzzer.playFromProgramSpace(PSTR("!>>a32"));
}

//Method used to reverse the directions stored in 
//Path array
char reverseDirection(char dir)
{
  switch(dir)
  {
    case 'B':
    return 'B';

    case 'S':
    return 'S';

    case 'L':
    return 'R';

    case 'R':
    return 'L'; 
  }
}


//Method used to return home after locating the house
void returnHome()
{
    turnAroundComeHome();
    while(1)
    {            
      if(pathLength == 0)
      {
        //You have Returned Home
        motors.setSpeeds(0, 0);
        break;
      }

      //driveToIntersectionCenter();
      Path[pathLength] = returnDirection;
      Times[timesLength] = returnTime;
      Serial.print(returnDirection);
      pathLength--;  
      timesLength--;    
      Serial.print(pathLength);

      if(reverseDirection(returnDirection) == 'S')
      {
        motors.setSpeeds(100, 100);
        delay(returnTime);
        break;
      }
      else if (reverseDirection(returnDirection) == 'B')
      {
        motors.setSpeeds(-100, -100); 
        delay(returnTime);
        break;
      }
      else if (reverseDirection(returnDirection) == 'L')
      {    
        motors.setSpeeds(-100, 100);
        delay(1250);
        stop();
        break;
      }
      else if (reverseDirection(returnDirection) == 'R')
      {        
        motors.setSpeeds(100, -100);
        delay(1250);
        stop();
        break;
      }      
    }
}

void loop()
{
  // Get the position of the line.  Note that we *must* provide
  // the "lineSensorValues" argument to readLine() here, even
  // though we are not interested in the individual sensor
  // readings.
  int16_t position = lineSensors.readLine(lineSensorValues, QTR_EMITTERS_ON, 1);

  // Our "error" is how far we are away from the center of the
  // line, which corresponds to position 2000.
  int16_t error = position - 2000;

  // Get motor speed difference using proportional and derivative
  // PID terms (the integral term is generally not very useful
  // for line following).  Here we are using a proportional
  // constant of 1/4 and a derivative constant of 6, which should
  // work decently for many Zumo motor choices.  You probably
  // want to use trial and error to tune these constants for your
  // particular Zumo and line course.
  int16_t speedDifference = error / 4 + 6 * (error - lastError);

  lastError = error;

  // Get individual motor speeds.  The sign of speedDifference
  // determines if the robot turns left or right.
  int16_t leftSpeed = (int16_t)maxSpeed + speedDifference;
  int16_t rightSpeed = (int16_t)maxSpeed - speedDifference;

  // Constrain our motor speeds to be between 0 and maxSpeed.
  // One motor will always be turning at maxSpeed, and the other
  // will be at maxSpeed-|speedDifference| if that is positive,
  // else it will be stationary.  For some applications, you
  // might want to allow the motor speed to go negative so that
  // it can spin in reverse.
  leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);
  

  //If a House hasn't been found yet then we continue our search
  //Using the left hand maze solving rule.
  if(housesToFind < housesFound)
  {
  forward();
  start = time(0);
  leftHandMazeSolve();
  }
  
  if(housesToFind == housesFound)
  {
    //We Have Found the House Time to return to the start
    buzzer.playFromProgramSpace(PSTR("!>>a32"));
    returnHome();
  }

}
