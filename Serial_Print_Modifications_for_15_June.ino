/*
   Heather's modifications for finding and printing averages, maximums, and even readings
*/

const int numberOfReadings = 600;
int measurements[numberOfReadings] ;
const int ldrPin = A0;

void setup() {
  Serial.begin(9600);
}


void loop() {

  for (int readings = 0; readings < numberOfReadings; readings++) {
    measurements[readings] = analogRead(ldrPin);
    delay(1);
  }


  // calculate the average
  int sum = 0;

  for (int readings = 0; readings < numberOfReadings; readings++) {
    sum = sum + measurements[readings];
  }

  int average = sum / numberOfReadings;
  Serial.print ("average of readings is ");
  Serial.println (average);
  

  // what's the difference between print() and println()?
  Serial.print("the maximum reading is ");
  Serial.println(measurements[highestReading()]);

  Serial.println (measurements[giveEvenReadings ()]);
}


//The following section is used to define the functions used in the loop

// return the index of the highest reading
// if there are multiple readings return one of them
int highestReading() {

  int latestHighestReading = 0;

  for (int readings = 0; readings < numberOfReadings; readings++)
  {
    if (measurements[readings] > measurements[latestHighestReading])
    {
      latestHighestReading = readings;
    }
  }
  return latestHighestReading;

}

//print only even numbers from array
int giveEvenReadings (){
 
  int evenReading = 0;

  for (int readings = 0; readings < numberOfReadings; readings ++)
  {
    if (measurements [readings] == readings + 2 ) 
    {
     evenReading = readings;
    }
   
  }
  return evenReading;
}

