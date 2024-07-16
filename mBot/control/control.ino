/**
 * @file    control.ino
 * @author  Guillaume Gibert
 * @version V0.1
 * @date    2021/04/18
 * @brief   Description: this code drives the mbot robot to a target location (x, y, theta) using the gyro data.
 *
 * Function List:
 * 1. void control::move(int, int)
 * 2. void control::stop(void) 
 *
 */
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "MeMCore.h"

// robot characteristics
double _wheelDiameter = 6.0; //cm
double _distanceBetweenWheels = 11.5; //cm
int _basicMotorSpeed = 100;

// Vars to change setup
double _targetPoints[] = {0.0, 0.0, 25.0, 0.0, 25.0, 25.0, 0.0, 25.0, 0.0, 0.0};
int _pointNumber = 0;
double _xTarget = 25.0;
double _yTarget = -25.0;
double _Kp_angle = -5.0;
double _errorAngleThreshold = 3.0; // in deg
double _distanceDurationFactor = 10.0; // at speed=100, robot moves about 10cm / s

// declare motors
MeDCMotor _leftMotor(9);
MeDCMotor _rightMotor(10);

// declare gyro
MeGyro _gyro;

// position, baseline...
double _xCurrent = 0.0;
double _yCurrent = 0.0;
double _thetaCurrent = 0.0;
double _thetaBaseline = 0.0;
double _errorAngle = 0.0; // in deg


void setup()
{
  // initialize the serial port
  Serial.begin(115200);

  // set no speed for left and right motors 
  _leftMotor.run((9)==M1?-(0):(0));
  _rightMotor.run((10)==M1?-(0):(0));

  // initialize the gyro
  _gyro.begin();

  // wait for 2 seconds
  delay(2000);

  // retrieve new values from gyro
  _gyro.update();

  // record the baseline for the theta angle (Rz)
  _thetaBaseline = _gyro.getAngleZ();

}

void loop()
{
  // choose next point
  if (_pointNumber < 5)
  {
    _xTarget = _targetPoints[2*_pointNumber];
    _yTarget = _targetPoints[2*_pointNumber+1];
    _pointNumber++;
    Serial.println("###NEW POINT###");
    Serial.print("(x, y) = (");
    Serial.print(_xTarget);
    Serial.print("(, ");
    Serial.print(_yTarget);
    Serial.println(" )");

    /////////////////
    // rotate first
  
    // determine the target angle
    double targetAngle = atan2(_yTarget-_yCurrent, _xTarget-_xCurrent)*180.0/PI;
    Serial.println("-->rotate");
    // adjust the angle
    _errorAngle = targetAngle - _thetaCurrent;
 
    while (abs(_errorAngle) > _errorAngleThreshold)
    {
      // determine the command (proportionnal controller) i.e. angular velocity
      double v = 0.0;
      double w = _Kp_angle * _errorAngle;
    
      // compute the command
      double leftWheelSpeed  =  v / (_wheelDiameter/2) - _distanceBetweenWheels * w / (_wheelDiameter) ;
      double rightWheelSpeed  = v / (_wheelDiameter/2) + _distanceBetweenWheels * w / (_wheelDiameter) ;
    
      // crop wheel speed
      if (leftWheelSpeed > 250)
        leftWheelSpeed = 250;
      if (rightWheelSpeed > 250)
        rightWheelSpeed = 250;
     
      // send the commands
      move(rightWheelSpeed, leftWheelSpeed);
    
      // retrieve new values from gyro
      _gyro.update();
    
      // get the theta angle (Rz)
      _thetaCurrent = _gyro.getAngleZ() - _thetaBaseline;
      _errorAngle = targetAngle - _thetaCurrent;
      //Serial.println(_errorAngle);
    }

    //////////////////
    // move forward after
    Serial.println("-->move forward");
    // determine the target distance
    double targetDistance = sqrt( pow(_xTarget-_xCurrent, 2) + pow(_yTarget-_yCurrent, 2) );

    //Serial.println(targetDistance);
  
    // adjust the position
    unsigned long _durationTranslation = targetDistance/_distanceDurationFactor*1000000; // 1 s = 10cm
  
    // record the start time
    unsigned long startTime = micros(); // in microseconds
    unsigned long currentTime = micros(); // in microseconds
  
    while (currentTime - startTime < _durationTranslation)
    {
      move(_basicMotorSpeed, _basicMotorSpeed);
      currentTime = micros(); 
    }

    _xCurrent = _xTarget;
    _yCurrent = _yTarget; 
    }
    else
      stop();

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
