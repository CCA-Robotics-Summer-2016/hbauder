
/*
  Heather's code for the ultrasonic distance-sensing robot that avoids obstacles
  modificaions based on M. Shiloh's modifications to Abrar's original code
*/

//Control Pins for the left half of H-bridge
const int enable1 = 6; // PWM pin for speed control
const int in1 = 4;
const int in2 = 2;

// Control pins for the right half of the H-bridge
const int enable2 = 5; // PWM pin for speed control
const int in3 = 8;
const int in4 = 7;

//Control pins for ultrasonic distance sensor
const int trigPin = 13;
const int echoPin = 11;

//Set Minimum Distance required for Bot to maintain from obstacles (in cm)
const int minimumDistance = 20;

void setup() {

  //control pins for ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //control pins for the left motor
  pinMode( enable1, OUTPUT);
  pinMode( in1, OUTPUT);
  pinMode( in2, OUTPUT);

  //control pins for the right motor
  pinMode( enable2, OUTPUT);
  pinMode( in3, OUTPUT);
  pinMode( in4, OUTPUT);

  Serial.begin(9600);

}

void loop() {

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

//The section that follows is used to define the functions used in this code loop

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

  long measureDistance () {
    long duration, distance;

    // send the pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2); // low for 2 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10); // high for 10 microseconds
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH); // measure the time to the echo
    distance = (duration / 2) / 29.1; // calculate the distance in cm
    return distance;
  }

