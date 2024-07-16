#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeMCore.h>


// declare motors
MeDCMotor _leftMotor(9);
MeDCMotor _rightMotor(10);

// declare motor speeds
int _leftMotorSpeed = 0;
int _rightMotorSpeed = 0;

void setup()
{
  Serial.begin(9600);
  // set no speed for left and right motors 
  _leftMotor.run((9)==M1?-(0):(0));
  _rightMotor.run((10)==M1?-(0):(0));
}

void loop()
{
  // Move forward
  Serial.println("Move forward");
  move(100, 100, 2000);
 
  // Move backwards
  Serial.println("Move backward");
  move(-100, -100, 2000);
  
  // Turn right
  Serial.println("Turn right");
  move(100, -100, 2000);

  // Turn left
  Serial.println("Turn left");
  move(-100, 100, 2000);
 
}

void move(int leftMotorSpeed, int rightMotorSpeed, int duration)
{
  _leftMotorSpeed = leftMotorSpeed;
  _rightMotorSpeed = rightMotorSpeed;
  
  _leftMotor.run((9)==M1?-(_leftMotorSpeed):(_leftMotorSpeed));
  _rightMotor.run((10)==M1?-(_rightMotorSpeed):(_rightMotorSpeed));
  delay(duration);
  _leftMotor.stop();
  _rightMotor.stop();
  
}
