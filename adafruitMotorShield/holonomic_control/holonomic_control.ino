#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

const bool _verboseMode = true;
const int _portBaud = 9600; // baud rate of the serial port
const float _fps = 10.0; // frame rate to send data
const int _topleftMotorSpeed = 150; 
const int _toprightMotorSpeed = 150; 
const int _bottomleftMotorSpeed = 150; 
const int _bottomrightMotorSpeed = 150; 

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *topleftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *toprightMotor = AFMS.getMotor(2);
Adafruit_DCMotor *bottomleftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *bottomrightMotor = AFMS.getMotor(4);

void setup()
{
  // initializes the serial port
  Serial.begin(_portBaud);

  // launches the motor control via the shield
  AFMS.begin();

  // sets the motor speeds
  topleftMotor->setSpeed(_topleftMotorSpeed);
  toprightMotor->setSpeed(toprightMotor);
  bottomleftMotor->setSpeed(_bottomleftMotorSpeed);
  bottomrightMotor->setSpeed(bottomrightMotor);
}


void loop()
{
  // initialzes the timer
  unsigned long startTime = micros();

  //TODO 
   
  // estimates the loop duration and waits before starting a new loop
  unsigned long currentTime = micros(); // in microseconds
  unsigned long elapsedTime = currentTime - startTime; // in microseconds
  unsigned long waitingTime = 1000000/_fps - elapsedTime; // in microseconds
  
  if (waitingTime < 16384) //(see https://www.arduino.cc/reference/en/language/functions/time/delaymicroseconds/)
    delayMicroseconds(waitingTime); // in microseconds
  else
    delay(waitingTime / 1000); // in milliseconds 
 
}

void moveStop()
{
  if (_verboseMode)
    Serial.println("Stop");
    
  topleftMotor->run(RELEASE);
  toprightMotor->run(RELEASE);
  bottomleftMotor->run(RELEASE);
  bottomrightMotor->run(RELEASE);
}

void moveForward()
{
  if (_verboseMode)
    Serial.println("Forward");
    
  topleftMotor->run(FORWARD);
  toprightMotor->run(FORWARD);
  bottomleftMotor->run(FORWARD);
  bottomrightMotor->run(FORWARD);
}

void moveForwardRight()
{
  if (_verboseMode)
    Serial.println("Forward Right");
    
  topleftMotor->run(FORWARD);
  toprightMotor->run(RELEASE);
  bottomleftMotor->run(RELEASE);
  bottomrightMotor->run(FORWARD);
}

void moveRight()
{
  if (_verboseMode)
    Serial.println("Right");
    
  topleftMotor->run(FORWARD);
  toprightMotor->run(BACKWARD);
  bottomleftMotor->run(BACKWARD);
  bottomrightMotor->run(FORWARD);
}

void moveBackwardRight()
{
  if (_verboseMode)
    Serial.println("Backward Right");
    
  topleftMotor->run(BACKWARD);
  toprightMotor->run(RELEASE);
  bottomleftMotor->run(RELEASE);
  bottomrightMotor->run(BACKWARD);
}

void moveBackward()
{
  if (_verboseMode)
    Serial.println("Backward");
    
  topleftMotor->run(BACKWARD);
  toprightMotor->run(BACKWARD);
  bottomleftMotor->run(BACKWARD);
  bottomrightMotor->run(BACKWARD);
}

void moveBackwardLeft()
{
  if (_verboseMode)
    Serial.println("Backward Left");
    
  topleftMotor->run(RELEASE);
  toprightMotor->run(BACKWARD);
  bottomleftMotor->run(BACKWARD);
  bottomrightMotor->run(RELEASE);
}

void moveLeft()
{
  if (_verboseMode)
    Serial.println("Left");
    
  topleftMotor->run(BACKWARD);
  toprightMotor->run(FORWARD);
  bottomleftMotor->run(FORWARD);
  bottomrightMotor->run(BACKWARD);
}

void moveForwardLeft()
{
  if (_verboseMode)
    Serial.println("Forward Left");
    
  topleftMotor->run(RELEASE);
  toprightMotor->run(FORWARD);
  bottomleftMotor->run(FORWARD);
  bottomrightMotor->run(RELEASE);
}

void spinRight()
{
  if (_verboseMode)
    Serial.println("Spin Right");
    
  topleftMotor->run(FORWARD);
  toprightMotor->run(BACKWARD);
  bottomleftMotor->run(FORWARD);
  bottomrightMotor->run(BACKWARD);
}

void spinLeft()
{
  if (_verboseMode)
    Serial.println("Spin Left");
    
  topleftMotor->run(BACKWARD);
  toprightMotor->run(FORWARD);
  bottomleftMotor->run(BACKWARD);
  bottomrightMotor->run(FORWARD);
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
    
    switch (inChar)
    {
      case '8':
        moveForward(); break;
      case '9':
        moveForwardRight(); break;
      case '6':
        moveRight(); break;
      case '3':
        moveBackwardRight(); break;
      case '2':
        moveBackward(); break;
      case '1': 
        moveBackwardLeft(); break;
      case '4': 
        moveLeft(); break;
      case '7': 
        moveForwardLeft(); break;
      case '5': 
        spinLeft(); break;
      case '0': 
        moveStop(); break;
      default: 
        moveStop(); break;
    }
  }
}
