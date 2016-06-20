#include <Servo.h>

//Global array of measurements
long measurements [180];
long averages [18];

//the ultrasonic distance measuring sensor that sits on top of this servo
Servo myServo;
const int servoPin = 9;
int pos = 0; //vairiable to store the servo position

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

//Later on, Move These 2 to loop()
  //look around and see what's nearby
  sweepAndMeasure();

  //look at measurments in 10 degree chunks
  averageInTenDegreeChunks ();

}

void loop () {



  //finally decide where to go 
  
  
  
}



//The section that follows is used for defining the functions used in the loop


//sweep 180 degrees and populate the measurements array
void sweepAndMeasure() {
  Serial.println ("Hello from sweepAndMeasure()");
  for (int degree = 0; degree < 180; degree ++) {
    myServo.write (degree);
    delay (15);
    measurements [degree] = measureDistance();
  }
}

void averageInTenDegreeChunks () {
  Serial.println ("Hello from averageInTenDegreeChunks");
  for (int degree = 0; degree < 180; degree = degree + 10) {
   averages[degree/10] = averageTheseTen (degree);
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
      //skip thid measurement
      validMeasurements --;
    } else {
    sum = sum + measurements [degree];
    }
  }
  average = sum / validMeasurements;
  
  Serial.print ("average: ");
  Serial.print (average);
  Serial.print ("\t");
  Serial.print ("Valid Measurements: ");
  Serial.print (validMeasurements);
  Serial.println ("");
  
  return (average);
}

// Take a measurement using the ultrasonic discance
// measuring sensor and return the distance in cm
// no error checking takes place

long measureDistance() {
  Serial.println ("Hello from measureDistance()");
  
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
  return distance;

  //if reading is out of range, make it -1
  if (distance <= 0 || distance >= 200) {
    distance = -1;
  }
  
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
