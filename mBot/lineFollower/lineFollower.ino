/**
 * @file    lineFollower.ino
 * @author  Guillaume Gibert
 * @version V0.1
 * @date    2021/04/21
 * @brief   Description: this code drives the robot to follow a line.
 *
 * Function List:
 * 1. uint8_t lineFollower::readSensors(void)
 *
 */
#include "MeMCore.h"

// vars to change
double _Kp_angle = -5.0;
double _speedIncrement = 1.0;
double _translationalSpeed = 0.0;
const float _fps = 1000.0; // frame rate to send data

// declare line follower sensor
MeLineFollower lineFinder(PORT_2); /* Line Finder module can only be connected to PORT_3, PORT_4, PORT_5, PORT_6 of base shield. */

// declare motors
MeDCMotor _leftMotor(9);
MeDCMotor _rightMotor(10);

// robot characteristics
double _wheelDiameter = 6.0; //cm
double _distanceBetweenWheels = 11.5; //cm
int _basicMotorSpeed = 80;

void setup()
{
  // initialize the serial port
  Serial.begin(9600);

  // set no speed for left and right motors 
  _leftMotor.run((9)==M1?-(0):(0));
  _rightMotor.run((10)==M1?-(0):(0));

  _translationalSpeed = _basicMotorSpeed;
}

void loop()
{
  // record the start time
  unsigned long startTime = micros(); // in microseconds
  
  // set the basic command to go straight
  double v = _basicMotorSpeed * (_wheelDiameter/2);
  double w = 0.0;

  // read the sensor
  int sensorState = lineFinder.readSensors();
  switch(sensorState)
  {
    case S1_IN_S2_IN: 
      Serial.println("Sensor 1 and 2 are inside of black line"); 
      //_translationalSpeed += _speedIncrement;
      _Kp_angle = 0.0;
      break;
    case S1_IN_S2_OUT: 
      Serial.println("Sensor 2 is outside of black line"); 
      _Kp_angle = 1.0;
      //_translationalSpeed -= _speedIncrement;
      break;
    case S1_OUT_S2_IN: 
      Serial.println("Sensor 1 is outside of black line");
      _Kp_angle = -1.0; 
      //_translationalSpeed -= _speedIncrement;
      break;
    case S1_OUT_S2_OUT: 
      Serial.println("Sensor 1 and 2 are outside of black line");
      _Kp_angle = 0.0;
      //_translationalSpeed -= _speedIncrement;      
      break;
    default: 
      break;
  }

  //  
  w = _Kp_angle * _basicMotorSpeed;
  v = _translationalSpeed * (_wheelDiameter/2);
  move(v, w);
/*
  unsigned long currentTime = micros(); // in microseconds
  unsigned long elapsedTime = currentTime - startTime; // in microseconds
  unsigned long waitingTime = 1000000.0/_fps - elapsedTime; // in microseconds
  
  // wait to keep a constant framerate
  if (waitingTime < 16384) //(see https://www.arduino.cc/reference/en/language/functions/time/delaymicroseconds/)
    delayMicroseconds(waitingTime); // in microseconds
  else
    delay(waitingTime / 1000); // in milliseconds 
    */
}


void move(double v, double w)
{
  // compute the command
  double leftWheelSpeed  =  v / (_wheelDiameter/2) - _distanceBetweenWheels * w / (_wheelDiameter) ;
  double rightWheelSpeed  = v / (_wheelDiameter/2) + _distanceBetweenWheels * w / (_wheelDiameter) ;
    
  // crop wheel speed
  if (leftWheelSpeed > 250)
    leftWheelSpeed = 250;
  if (rightWheelSpeed > 250)
    rightWheelSpeed = 250;

  // send the commands
  _leftMotor.run((9)==M1?-(leftWheelSpeed):(leftWheelSpeed));
  _rightMotor.run((10)==M1?-(rightWheelSpeed):(rightWheelSpeed));
}

void stop()
{
  _leftMotor.stop();
  _rightMotor.stop();
}
