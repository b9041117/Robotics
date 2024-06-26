// GridMovement.h provides functions to help the Zumo navigate a
// grid made of black lines on a white surface.
//
// The code uses the line sensors to follow lines and detect
// intersections, and it uses the gyro to help perform turns.
//
// This code was designed with maze-solving in mind, but it can
// be used in other applications as long as there are black
// lines on a white surface, and the lines are not too close
// together, and the lines intersect at right angles.
//
// The speed and delay parameters in this file were designed for
// a Zumo 32U4 with 4 NiMH batteries and 75:1 HP gearmotors.
// They might need to be adjusted depending on your motor and
// battery voltage.
//
// This file should be included once in your sketch, somewhere
// after you include TurnSensor.h and define objects named
// buttonA, display, imu, lineSensors, and motors.

#pragma once

#include <Wire.h>

// Motor speed when driving straight.  400 is the max speed.
const uint16_t straightSpeed = 90;

// The delay to use between first detecting an intersection and
// starting to turn.  During this time, the robot drives
// straight.  Ideally this delay would be just long enough to get
// the robot from the point where it detected the intersection to
// the center of the intersection.
const uint16_t intersectionDelay = 130;

// Motor speed when turning.  400 is the max speed.
const uint16_t turnSpeed = 200;

// Motor speed when turning during line sensor calibration.
const uint16_t calibrationSpeed = 200;

// This line sensor threshold is used to detect intersections.
const uint16_t sensorThresholdGM = 6;

// This line sensor threshold is used to detect the end of the
// maze.
const uint16_t sensorThresholdDark = 600;

// The number of line sensors we are using.
const uint8_t numSensors = 3;

// For angles measured by the gyro, our convention is that a
// value of (1 << 29) represents 45 degrees.  This means that a
// uint32_t can represent any angle between 0 and 360.
const int32_t gyroAngle45 = 0x20000000;

uint16_t lineSensorValuesGM[numSensors];


// Sets up special characters for the display so that we can show
// bar graphs.
static void loadCustomCharacters()
{
  static const char levels[] PROGMEM = {
    0, 0, 0, 0, 0, 0, 0, 63, 63, 63, 63, 63, 63, 63
  };
  display.loadCustomCharacter(levels + 0, 0);  // 1 bar
  display.loadCustomCharacter(levels + 1, 1);  // 2 bars
  display.loadCustomCharacter(levels + 2, 2);  // 3 bars
  display.loadCustomCharacter(levels + 3, 3);  // 4 bars
  display.loadCustomCharacter(levels + 4, 4);  // 5 bars
  display.loadCustomCharacter(levels + 5, 5);  // 6 bars
  display.loadCustomCharacter(levels + 6, 6);  // 7 bars
}

void printBar(uint8_t height)
{
  if (height > 8) { height = 8; }
  const char barChars[] = {' ', 0, 1, 2, 3, 4, 5, 6, (char)255};
  display.print(barChars[height]);
}

// Takes calibrated readings of the lines sensors and stores them
// in lineSensorValues.  Also returns an estimation of the line
// position.
uint16_t readSensors()
{
  return lineSensors.readLine(lineSensorValuesGM, QTR_EMITTERS_ON, 1);
}

// Returns true if the sensor is seeing a line.
// Make sure to call readSensors() before calling this.
bool aboveLine(uint8_t sensorIndex)
{
  return lineSensorValuesGM[sensorIndex] > sensorThresholdGM;
}

// Returns true if the sensor is seeing a lot of darkness.
// Make sure to call readSensors() before calling this.
bool aboveLineDark(uint8_t sensorIndex)
{
  return lineSensorValuesGM[sensorIndex] > sensorThresholdDark;
}

// Checks to see if we are over a dark spot, like the ones used
// to mark the end of a maze.  If all the middle sensors are over
// dark black, that means we have found the spot.
// Make sure to call readSensors() before calling this.
bool aboveDarkSpot()
{
  return aboveLineDark(1) && aboveLineDark(2) && aboveLineDark(3);
}

// Turns according to the parameter dir, which should be 'L'
// (left), 'R' (right), 'S' (straight), or 'B' (back).  We turn
// most of the way using the gyro, and then use one of the line
// sensors to finish the turn.  We use the inner line sensor that
// is closer to the target line in order to reduce overshoot.
void turn(char dir)
{
  if (dir == 'S')
  {
    // Don't do anything!
    return;
  }

  turnSensorReset();

  uint8_t sensorIndex;

  switch(dir)
  {
  case 'B':
    // Turn left 125 degrees using the gyro.
    motors.setSpeeds(-turnSpeed, turnSpeed);
    while((int32_t)turnAngle < turnAngle45 * 3)
    {
      turnSensorUpdate();
    }
    sensorIndex = 1;
    break;

  case 'L':
    // Turn left 45 degrees using the gyro.
    motors.setSpeeds(-turnSpeed, turnSpeed);
    while((int32_t)turnAngle < turnAngle45)
    {
      turnSensorUpdate();
    }
    sensorIndex = 1;
    break;

  case 'R':
    // Turn right 45 degrees using the gyro.
    motors.setSpeeds(turnSpeed, -turnSpeed);
    while((int32_t)turnAngle > -turnAngle45)
    {
      turnSensorUpdate();
    }
    sensorIndex = 3;
    break;

  default:
    // This should not happen.
    return;
  }

  // Turn the rest of the way using the line sensors.
  while(1)
  {
     readSensors();
     if (aboveLine(sensorIndex))
     {
       // We found the line again, so the turn is done.
       break;
     }
  } 
}

// This function causes the robot to follow a line segment until
// it detects an intersection, a dead end, or a dark spot.
void followSegment()
{
  while(1)
  {
    // Get the position of the line.
    uint16_t position = readSensors();

    // Our "error" is how far we are away from the center of the
    // line, which corresponds to position 2000.
    int16_t error = (int16_t)position - 2000;

    // Compute the difference between the two motor power
    // settings, leftSpeed - rightSpeed.
    int16_t speedDifference = error / 4;

    // Get individual motor speeds.  The sign of speedDifference
    // determines if the robot turns left or right.
    int16_t leftSpeed = (int16_t)straightSpeed + speedDifference;
    int16_t rightSpeed = (int16_t)straightSpeed - speedDifference;

    // Constrain our motor speeds to be between 0 and straightSpeed.
    leftSpeed = constrain(leftSpeed, 0, (int16_t)straightSpeed);
    rightSpeed = constrain(rightSpeed, 0, (int16_t)straightSpeed);

    motors.setSpeeds(leftSpeed, rightSpeed);

    // We use the inner four sensors (1, 2, 3, and 4) for
    // determining whether there is a line straight ahead, and the
    // sensors 0 and 5 for detecting lines going to the left and
    // right.
    //
    // This code could be improved by skipping the checks below
    // if less than 200 ms has passed since the beginning of this
    // function.  Maze solvers sometimes end up in a bad position
    // after a turn, and if one of the far sensors is over the
    // line then it could cause a false intersection detection.

    if(!aboveLine(0) && !aboveLine(1) && !aboveLine(2) && !aboveLine(3) && !aboveLine(4))
    {
      // There is no line visible ahead, and we didn't see any
      // intersection.  Must be a dead end.
      break;
    }

    if(aboveLine(0) || aboveLine(4))
    {
      // Found an intersection or a dark spot.
      break;
    }
  }
}

// This should be called after followSegment to drive to the
// center of an intersection.
void driveToIntersectionCenter()
{
  // Drive to the center of the intersection.
  motors.setSpeeds(straightSpeed, straightSpeed);
  delay(intersectionDelay);
}

// This should be called after followSegment to drive to the
// center of an intersection.  It also uses the line sensors to
// detect left, straight, and right exits.


// Calibrates the line sensors by turning left and right, then
// shows a bar graph of calibrated sensor readings on the display.
// Returns after the user presses A.
static void lineSensorSetup()
{
  display.clear();
  display.print(F("Line cal"));

  // Delay so the robot does not move while the user is still
  // touching the button.
  delay(1000);

  // We use the gyro to turn so that we don't turn more than
  // necessary, and so that if there are issues with the gyro
  // then you will know before actually starting the robot.

  turnSensorReset();

  // Turn to the left 90 degrees.
  motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
  while((int32_t)turnAngle < turnAngle45 * 2)
  {
    lineSensors.calibrate();
    turnSensorUpdate();
  }

  // Turn to the right 90 degrees.
  motors.setSpeeds(calibrationSpeed, -calibrationSpeed);
  while((int32_t)turnAngle > -turnAngle45 * 2)
  {
    lineSensors.calibrate();
    turnSensorUpdate();
  }

  // Turn back to center using the gyro.
  motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
  while((int32_t)turnAngle < 0)
  {
    lineSensors.calibrate();
    turnSensorUpdate();
  }

  // Stop the motors.
  motors.setSpeeds(0, 0);

  // Show the line sensor readings on the display until button A is
  // pressed.
  display.clear();
  while(!buttonA.getSingleDebouncedPress())
  {
    readSensors();

    display.gotoXY(0, 0);
    for (uint8_t i = 0; i < numSensors; i++)
    {
      uint8_t barHeight = map(lineSensorValuesGM[i], 0, 1000, 0, 8);
      printBar(barHeight);
    }
  }

  display.clear();
}

void gridMovementSetup()
{
  // Configure the pins used for the line sensors.
  lineSensors.initThreeSensors();

  // Set up custom characters on the display so we can show a bar
  // graph of the sensor readings after calibration.
  loadCustomCharacters();

  // Calibrate the gyro and show readings from it until the user
  // presses button A.
  turnSensorSetup();

  // Calibrate the sensors by turning left and right, and show
  // readings from it until the user presses A again.
  lineSensorSetup();
}
