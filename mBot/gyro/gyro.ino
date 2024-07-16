/**
 * @file    gyro.ino
 * @author  Guillaume Gibert
 * @version V0.1
 * @date    2021/04/17
 * @brief   Description: this code drives the mbot robot in straight line for a given duration and estimates its location (x, y, theta) from the gyro data.
 *
 * Function List:
 * 1. void MeGyro::move(int, int)
 * 2. void MeGyro::stop(void) 
 *
 */
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "MeMCore.h"

// Vars to change setup
int _basicMotorSpeed = 100;
const float _fps = 100.0; // frame rate to send data
unsigned long _duration = 5000000;
double _wheelDiameter = 6.0; //cm

// declare motors
MeDCMotor _leftMotor(9);
MeDCMotor _rightMotor(10);

// declare gyro
MeGyro _gyro;

// declare time
unsigned long _initTime = 0;
unsigned long _startTime = 0;

// speed, postion...
double _speed = 0.0;
double _position = 0.0;
double _x = 0.0;
double _y = 0.0;
double _positionIncrement = 0.0;

// baselines
int16_t _accYBaseline = 0;
double _thetaBaseline = 0.0;

// Conversion factor from accelerometer data to m/s² values
float _scale = (32.0/65536.0)*9.81*10.0; // scale factor +-16g (i.e. delta = 32g) on 16 bits (i.e. 65536 values) int converted in cm.s-2

void setup()
{
  // initialize the serial port
  Serial.begin(115200);

  // set no speed for left and right motors 
  _leftMotor.run((9)==M1?-(0):(0));
  _rightMotor.run((10)==M1?-(0):(0));

  // initialize the gyro
  _gyro.begin();

  // wait 2 seconds
  delay(2000);

  // retrieve new values from gyro
  _gyro.update();

  // record the baseline for the acceleration along -y and the theta angle (Rz)
  _accYBaseline = -_gyro.getAccY(); // baseline
  _thetaBaseline = _gyro.getAngleZ();

  // record the start time
  _initTime = micros(); // in microseconds

}

void loop()
{
  // record the start time
  _startTime = micros(); // in microseconds

  // retrieve new values from gyro
  _gyro.update();
  
  // print on the serial port
  Serial.print(_speed );
  Serial.print(",");
  Serial.print(_x );
  Serial.print(",");
  Serial.println(_y );
  
  unsigned long currentTime = micros(); // in microseconds
 
  if (currentTime - _initTime < _duration) // check if the duration threshold has been exceeded
  {
    // move straight
    move(_basicMotorSpeed, _basicMotorSpeed);
  }
  else
  {
    // stop the robot, time is over!
    stop();
  }

  // estimate speed from acceleration along -y by integration
    _speed += (-_gyro.getAccY() -_accYBaseline)*_scale*1.0/_fps;
    // estimate position increment along x-axis of the robot reference frame
    _position = _speed*1.0/_fps;
    
    double runningSpeed = (double)_basicMotorSpeed/255*3.7/6*200.0/60; // motor speed encoded on 255 values, average speed 200rpm 
    _positionIncrement = runningSpeed*1.0/_fps*3.14*_wheelDiameter;

    // get the theta angle (Rz)
    double currentTheta = _gyro.getAngleZ() - _thetaBaseline;

    // update the (x,y) coordinates of the robot
    _x += _position * cos(currentTheta/180.0*PI);
    _y += _position * sin(currentTheta/180.0*PI);

  currentTime = micros(); // in microseconds
  unsigned long elapsedTime = currentTime - _startTime; // in microseconds
  unsigned long waitingTime = 1000000.0/_fps - elapsedTime; // in microseconds
  
  // wait to keep a constant framerate
  if (waitingTime < 16384) //(see https://www.arduino.cc/reference/en/language/functions/time/delaymicroseconds/)
    delayMicroseconds(waitingTime); // in microseconds
  else
    delay(waitingTime / 1000); // in milliseconds 
}

void move(int leftMotorSpeed, int rightMotorSpeed)
{
  _leftMotor.run((9)==M1?-(leftMotorSpeed):(leftMotorSpeed));
  _rightMotor.run((10)==M1?-(rightMotorSpeed):(rightMotorSpeed));
}

void stop()
{
  _leftMotor.stop();
  _rightMotor.stop();
}
