/*
  Heather's working code.

  20 June 2016: initial upload
  20 June 2016: in class edits based on mShiloh's additions to the code
  25 June 2016: edits to try to make the robot actually do something in response to the data
*/

//include the servo library
#include <Servo.h>

// add restrictions to the rotational distance that the servo turns
// keeps from breaking the servo at the extremes of it's range
// please chose multiples of ten or -ms
// the program may not work -ms
#define MIN_SERVO_VALUE 30
#define MAX_SERVO_VALUE 160
#define SERVO_RANGE (MAX_SERVO_VALUE - MIN_SERVO_VALUE)


//Global array of measurements
// These must be the full range -ms
// since index is absolute degree and not relative degree -ms
long measurements [180];
long averages [180 / 10];

//the ultrasonic distance measuring sensor that sits on top of this servo -ms
Servo myServo;
const int servoPin = 9;
int pos = 0; //vairiable to store the servo position -ms does not have this anymore; is this necessary?
const int leftDegree = 150;
const int rightDegree = 30;
const int centerDegree = 90;

// ultrasonic distance measuring sensor
const int trigPin = 12;
const int echoPin = 11;
const int minimumDistance = 20;

// light sensors
const int leftLDR = A0;
const int rightLDR = A1;

// Control pins for the right half of the H-bridge
const int enable2 = 5; // PWM pin for speed control
const int in3 = 8;
const int in4 = 7;

//other half
const int enable1 = 6; // PWM pin for speed control
const int in1 = 4;
const int in2 = 2;

void setup() {

  Serial.begin (9600);
  Serial.println ("hello, my name is Inigo Montoya, version 0.1");

  // pins for the ultrasonic distance measuring sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //pin for the servo
  myServo.attach(servoPin);  // attaches the servo on pin 9 to the servo object
  myServo.write (0); //sets the servo to start at 0 degrees

  // motors
  pinMode( enable1, OUTPUT);
  pinMode( in1, OUTPUT);
  pinMode( in2, OUTPUT);

  pinMode( enable2, OUTPUT);
  pinMode( in3, OUTPUT);
  pinMode( in4, OUTPUT);



}



void loop() {

  Serial.println ("Hello from leftDistance()");
  myServo.write (leftDegree);
  delay (15);
  int leftDistance = measureDistance();
  Serial.print ("leftDistance = ");
  Serial.print (leftDistance);
  Serial.println (" cm");
  delay (500);

  Serial.println ("Hello from centerDistance()");
  myServo.write (centerDegree);
  delay (15);
  int centerDistance = measureDistance();
  Serial.print ("centerDistance = ");
  Serial.print (centerDistance);
  Serial.println (" cm");
  delay (500);

  Serial.println ("Hello from rightDistance()");
  myServo.write (rightDegree);
  delay (15);
  int rightDistance = measureDistance();
  Serial.print ("rightDistance = ");
  Serial.print (rightDistance);
  Serial.println (" cm");
  delay (500);

  if (centerDistance <= minimumDistance) {
    goReverse (500);
    stopRobot (250);
  }


  //Left distance is greater than both center and right distances
  //Left LDR is greater
  if (leftDistance > centerDistance
      && leftDistance > rightDistance
      && analogRead (leftLDR) > analogRead (rightLDR) ) {
    turnLeft (500);
  }

  //Left distance is greater than both center and right distances
  //Right LDR is greater
  if (leftDistance > centerDistance
      && leftDistance > rightDistance
      && analogRead (leftLDR) < analogRead (rightLDR) ) {
    turnLeft (250);
    goForward (500);
  }

  //Center distance is greater than both left and right distances
  //Left LDR is greater
  if (centerDistance > leftDistance
      && centerDistance > rightDistance
      && analogRead (leftLDR) > analogRead (rightLDR) ) {
    turnLeft (250);
    goForward (500);
  }

  //Center distance is greater than both left and right distances
  //right LDR is greater
  if (centerDistance > leftDistance
      && centerDistance < rightDistance
      && analogRead (leftLDR) > analogRead (rightLDR) ) {
    turnRight (250);
    goForward (500);
  }

  //Right distance is greater than both center and left distances
  //Left LDR is greater
  if (rightDistance > centerDistance
      && rightDistance > leftDistance
      && analogRead (leftLDR) > analogRead (rightLDR) ) {
    turnRight (250);
    goForward (500);
  }

  //Right distance is greater than both center and left distances
  //Right LDR is greater
  if (rightDistance > centerDistance
      && rightDistance > leftDistance
      && analogRead (leftLDR) < analogRead (rightLDR) ) {
    turnRight (500);
  }

  stopRobot (500);

}




//finally decide where to go






//***********//
//The section that follows is used for defining the functions used in the loop
//***********//


//take measurements at left, center, and right points
void threePointMeasure() {
  Serial.println ("Hello from threePointMeasure()");
  takeLeftDistance();
  takeCenterDistance();
  takeRightDistance();

}

//define function for measuing left distance
int takeLeftDistance() {
  Serial.println ("Hello from leftDistance()");
  myServo.write (leftDegree);
  delay (15);
  int leftDistance = measureDistance();
  Serial.print ("leftDistance = ");
  Serial.print (leftDistance);
  Serial.println (" cm");
  delay (500);

}

//define function for measuring right distance
int takeRightDistance() {
  Serial.println ("Hello from rightDistance()");
  myServo.write (rightDegree);
  delay (15);
  int rightDistance = measureDistance();
  Serial.print ("rightDistance = ");
  Serial.print (rightDistance);
  Serial.println (" cm");
  delay (500);
}

//define function for measuring center distance
int takeCenterDistance() {
  Serial.println ("Hello from centerDistance()");
  myServo.write (centerDegree);
  delay (15);
  int centerDistance = measureDistance();
  Serial.print ("centerDistance = ");
  Serial.print (centerDistance);
  Serial.println (" cm");
  delay (500);
}

//sweep 180 degrees and populate the measurements array
void sweepAndMeasure() {
  Serial.println ("Hello from sweepAndMeasure()");
  for (int degree = MIN_SERVO_VALUE; degree < MAX_SERVO_VALUE; degree++) {
    myServo.write (degree);
    delay (15);
    measurements [degree] = measureDistance();
  }
}

void averageInTenDegreeChunks () {
  Serial.println ("Hello from averageInTenDegreeChunks");
  for (int degree = MIN_SERVO_VALUE; degree < MAX_SERVO_VALUE; degree = degree + 10) {
    averages [degree / 10] = averageTheseTen (degree);
  }

}

int averageTheseTen (int startHere) {
  Serial.println ("Hello from averageTheseTen()");
  int sum = 0;
  int validMeasurements = 10;
  int average;
  for (int degree = startHere; degree < startHere + 10; degree ++) {
    Serial.print (measurements[degree]);
    Serial.print ("\t");
    if (-1 == measurements[degree] ) {
      //skip this measurement
      validMeasurements --;
    } else {
      sum = sum + measurements [degree];
    }
  }
  average = sum / validMeasurements;

  Serial.print ("average: ");
  Serial.print (average);
  Serial.print ("\t");
  Serial.print ("# of Valid Measurements: ");
  Serial.print (validMeasurements);
  Serial.println ("");

  return (average);
}

// Take a measurement using the ultrasonic discance
// measuring sensor and return the distance in cm
// no error checking takes place

long measureDistance() {
  Serial.println ("Hello from measureDistance()\t");

  long duration, distance;

  // measure how far anything is from us
  // send the pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2); // low for 2 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // high for 10 microseconds
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH); // measure the time to the echo
  distance = (duration / 2) / 29.1; // calculate the distance in cm

  //if reading is out of range, make it -1
  if (distance <= 0 || distance >= 200) {
    distance = -1;
  }
  return distance;

}


void goForward (int timeToMove) {

  //Go Forward for a certain amount of time
  Serial.print ("Move forward for ");
  Serial.println (timeToMove); //both motors go forward

  digitalWrite (in1, HIGH);
  digitalWrite (in2, LOW);
  analogWrite (enable1, 150);
  digitalWrite (in3, HIGH);
  digitalWrite (in4, LOW);
  analogWrite (enable2, 150);
  delay (timeToMove); // wait for humans to enjoy the bot going forward
}

void turnLeft (int timeToMove) {
  Serial.print ("turn left for ");
  Serial.println (timeToMove);

  digitalWrite (in1, HIGH);
  digitalWrite (in2, LOW);
  analogWrite (enable1, 150);
  digitalWrite (in3, HIGH);
  digitalWrite (in4, LOW);
  analogWrite (enable2, 100);
  delay (timeToMove);
}

void turnRight (int timeToMove) {

  Serial.print ("turn right for ");
  Serial.println (timeToMove);

  digitalWrite (in1, HIGH);
  digitalWrite (in2, LOW);
  analogWrite (enable1, 100);
  digitalWrite (in3, HIGH);
  digitalWrite (in4, LOW);
  analogWrite (enable2, 150);
  delay (timeToMove);
}

void stopRobot (int timeToMove) {

  Serial.print ("stop for ");
  Serial.println (timeToMove); //both motors stop

  digitalWrite (in1, LOW);
  digitalWrite (in2, LOW);
  digitalWrite (enable1, LOW);
  digitalWrite (in3, LOW);
  digitalWrite (in4, LOW);
  digitalWrite (enable2, LOW);
  delay (timeToMove); // wait for humans to enjoy the bot stopping
}

void goReverse (int timeToMove) {

  //Go backward for a certain abmount of time
  Serial.print ("Move backwards for ");
  Serial.println (timeToMove); //both motors go in reverse

  digitalWrite (in1, LOW);
  digitalWrite (in2, HIGH);
  digitalWrite (enable1, HIGH);
  digitalWrite (in3, LOW);
  digitalWrite (in4, HIGH);
  digitalWrite (enable2, HIGH);
  delay (timeToMove); // wait for humans to enjoy the bot going in reverse
}
