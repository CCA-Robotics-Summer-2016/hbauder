/*
*Heather's working code.
*
*20 June 2016: initial upload 
*20 June 2016: in class edits based on mShiloh's additions to the code
*25 June 2016: edits to try to make the robot actually do something in response to the data
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

// ultrasonic distance measuring sensor
const int trigPin = 12;
const int echoPin = 11;

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

  // motors
  pinMode( enable1, OUTPUT);
  pinMode( in1, OUTPUT);
  pinMode( in2, OUTPUT);

  pinMode( enable2, OUTPUT);
  pinMode( in3, OUTPUT);
  pinMode( in4, OUTPUT);

//Later on, Move These 2 functions to loop()
  //look around and see what's nearby
  void sweepAndMeasure();

  //look at measurments in 10 degree chunks
  averageInTenDegreeChunks ();

}

void loop () {



  //finally decide where to go  
  //exampe from Obstacle Avoidance Robot, need to modify to use sweepAndMeasure() and averageInTenDegreeChunks()

  long distance;
  distance = measureDistance ();
  Serial.print (distance);
  Serial.println ("cm");

  // check validity of distance
  if (distance >= 200 || distance <= 0) {
    Serial.println("invalid values; heading into the unknown");
    goForward (500);
    stopRobot (300);
  }

  else  {
    if (distance < minimumDistance) {
      //turn left, take measurement
      //turn right, take measurement
      //go towards greater distance

      turnLeft (100);
      int leftDistance = measureDistance();
      turnRight (200);
      int rightDistance = measureDistance();
      turnLeft (100); //return to forward position

      if (leftDistance >= 200
          || leftDistance <= 0
          || rightDistance >= 200
          || rightDistance <= 0)
      {
        Serial.println ("Either Left or Right reading is bad");

      }
      else if (leftDistance > rightDistance) {
        turnLeft (100);
      }
      else {
        turnRight (100);
      }
    } //end of case for distance less than minimumDistance

    //begin case for valid range with distance greater than minimumDistance
    else  {
      Serial.println ("distance in acceptable range");
      goForward (500);
    }
  } //end valid range case
} //end loop
  
  
}



//The section that follows is used for defining the functions used in the loop


//sweep 180 degrees and populate the measurements array
void sweepAndMeasure() {
  Serial.println ("Hello from sweepAndMeasure()");
  for (int degree = MIN_SERVO_VALUE; degree < MAX_SERVO_VALUE; degree++){
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
  for (int degree = startHere; degree < startHere + 10; degree ++){
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
