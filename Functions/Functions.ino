#include <Servo.h>

byte long1 = 1;
byte short1 = 1;
byte long2 = 1;
byte short2 = 1;
byte front, back, left, right, left_front, left_rear, right_rear, right_front;
int speed_val;
void setup(void){}
void loop(void){}

float short_range_dist (int ADCval){
  //Replace with derived formula
  float dist = 11.724/ADCval *(5/1023.0) - 0.24;
  return dist;
}

float long_range_dist (int ADCval){
  //Replace with derived formula
  float dist = 20.9/ADCval *(5/1023.0);
  return dist;
}

//INITIALISING
void orientation (){
  // Check readings, one will be max range, lowest voltage as voltage is inverse to distance
  float diff = 10;
  float gain = 0.1;
  int long1ADC = analogRead(long1);
  int long2ADC = analogRead(long2);
  int short1ADC = analogRead(short1);
  int short2ADC = analogRead(short2);

//  minLong =  long1ADC < long2ADC ? long1 : long2;
//  minShort = short1ADC < short2ADC ? short1 : short2;

  //Set the orientation
  //Adjust thresholds
  if ((short1ADC < 100) || (long1ADC < 100)){
    front = long1;
    back = long2;
    left = short2;
    right = short1;
    left_front = 46;
    left_rear = 47;
    right_rear = 50;
    right_front = 51;
  } else {
    front = long2;
    back = long1;
    left = short1;
    right = short2;
    left_front = 50;
    left_rear = 51;
    right_rear = 46;
    right_front = 47;
  }

  //Tune to a small movement
  if ((short1ADC < 100) || (short2ADC < 100)){
    speed_val = 100;
    gain = 0.1;
  } else {
    speed_val = -100;
    gain = -0.1;
  }
  
  int backADC = analogRead(back);
  float priorBackDist = long_range_dist(backADC);
  int leftADC = analogRead(left);
  float priorLeftDist = short_range_dist(leftADC);

  while((diff > 0.5*2) || (-diff > 0.5*2)){
    cw();
    delay(10);
    backADC = analogRead(back);
    float backDist = long_range_dist(backADC);
    leftADC = analogRead(left);
    float leftDist = short_range_dist(leftADC);

    diff = priorBackDist + priorLeftDist - backDist - leftDist;
    speed_val = gain * diff;
    priorBackDist = backDist;
    priorLeftDist = leftDist;
  }
}

void cw(){}
