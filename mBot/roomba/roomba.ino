#include <SoftwareSerial.h>
#include <MeMCore.h>

// declare US sensor
MeUltrasonicSensor ultraSensor(PORT_3); /* Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */
// declare distance threshold
double _distanceThreshold = 10.0; // in cm

// declare motors
MeDCMotor _leftMotor(9);
MeDCMotor _rightMotor(10);
// declare motor speeds
int _leftMotorSpeed = 0;
int _rightMotorSpeed = 0;
int _basicMotorSpeed = 100;
long _maxDelay = 2000; // ms
long _samplingDelay = 100; // ms

void setup()
{
  Serial.begin(9600);

  // set no speed for left and right motors 
  _leftMotor.run((9)==M1?-(0):(0));
  _rightMotor.run((10)==M1?-(0):(0));
}

void loop()
{
  double currentDistance = ultraSensor.distanceCm();
  if (currentDistance <= _distanceThreshold)
  {
    stop();
    
    Serial.print("Distance is lower than thresold:  ");
    Serial.print(currentDistance);
    Serial.print(" / ");
    Serial.print(_distanceThreshold);
    Serial.println(" cm");

    long randDelay = random(-_maxDelay, _maxDelay); // choose a random delay between 0 and 2s

    if (randDelay < 0) // turn left
      move(-_basicMotorSpeed, _basicMotorSpeed);
    else // turn right
      move(_basicMotorSpeed, -_basicMotorSpeed);

    delay(abs(randDelay));
  }
  else
  {
    move(_basicMotorSpeed, _basicMotorSpeed);
  }
  
  delay(_samplingDelay); /* the minimal measure interval is 100 milliseconds */
}

void move(int leftMotorSpeed, int rightMotorSpeed)
{
  _leftMotorSpeed = leftMotorSpeed;
  _rightMotorSpeed = rightMotorSpeed;
  
  _leftMotor.run((9)==M1?-(_leftMotorSpeed):(_leftMotorSpeed));
  _rightMotor.run((10)==M1?-(_rightMotorSpeed):(_rightMotorSpeed));
 
}

void stop()
{
  _leftMotor.stop();
  _rightMotor.stop();
}
