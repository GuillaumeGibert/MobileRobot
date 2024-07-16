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

// Vars to change setup
double _Kp_angle = -5.0;
double _targetAngle = 90;
const long int _baudRate = 115200;

// robot characteristics
double _wheelDiameter = 6.0; //cm
double _distanceBetweenWheels = 11.5; //cm
int _basicMotorSpeed = 100;

// internal global variables
String _inputString = "";         // a String to hold incoming data
bool _stringComplete = false;  // whether the string is complete

// declare motors
MeDCMotor _leftMotor(9);
MeDCMotor _rightMotor(10);

// declare gyro
MeGyro _gyro;

// position, baseline...
double _thetaCurrent = 0.0;
double _thetaBaseline = 0.0;
double _errorAngle = 0.0; // in deg



void setup()
{
  // initialize the serial port
  Serial.begin(_baudRate);

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
    // checks if an update of Kp value is available
    if (_stringComplete) 
    {
      // updates the Kp value
      _Kp_angle = _inputString.toDouble(); 

      // resets the input string to accept new input from the serial port
      _inputString = "";
      _stringComplete = false;
    }
     
    // adjust the angle
    _errorAngle = _targetAngle - _thetaCurrent;
    
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
    _errorAngle = _targetAngle - _thetaCurrent;

    // for display
    Serial.print("Current_angle:"); Serial.print(_thetaCurrent);Serial.print(",");
    Serial.print("Target_angle:"); Serial.println(_targetAngle);
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

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() 
{
  while (Serial.available()) 
  {
    // gets the new byte:
    char inChar = (char)Serial.read();
    // adds it to the _inputString:
    _inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // does something about it:
    if (inChar == '\n') 
    {
      _stringComplete = true;
    }
  }
}
