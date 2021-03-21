#include <Servo.h>
#include <math.h>

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29

HardwareSerial *SerialCom;

//Default sensor pins
const byte left_front_sensor = A7;
const byte left_back_sensor = A4;
const byte back_sensor = A11;
const byte front_sensor = A15;

int speed_val;
float separationDist = 19; //19 cm between the two left sensors


//Set these values
int sensorPin = 12;             //define the pin that gyro is connected
int T = 100;                    // T is the time of one loop
int sensorValue = 0;            // read out value of sensor
float gyroSupplyVoltage = 5;    // supply voltage for gyro
float gyroZeroVoltage = 509;      // the value of voltage when gyro is zero
float gyroSensitivity = 0.007;  // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 1.5;  // because of gyro drifting, defining rotation angular velocity  less than                                                        // this value will not be ignored
float gyroRate = 0;             // read out value of sensor in voltage
float currentAngle = 0;         // current angle calculated by angular velocity integral on


void setup(void){
  stop();
  SerialCom = &Serial;
  SerialCom ->begin(115200);
  SerialCom ->println("Starting");
  enable_motors();
  //orientation();
  turn_90_gyro();
  stop();
}
void loop(void){}

float left_back_dist (void){
  //Derived formula
  int ADCval = analogRead(left_back_sensor);
  float dist = 1/(0.0005*ADCval - 0.0078);   //11.724/ADCval *(5/1023.0) - 0.24;
  return dist; //in cm
}

float left_front_dist (void){
  //Derived formula
  int ADCval = analogRead(left_front_sensor);
  float dist = 1/(0.0005*ADCval - 0.0101);   //11.724/ADCval *(5/1023.0) - 0.24;
  return dist; //in cm
}

float front_dist (void){
  //Derived formula
  int ADCval = analogRead(front_sensor);
  float dist = 1/(0.0002*ADCval - 0.0068);  //20.9/ADCval *(5/1023.0);
  return dist; //in cm
}

float back_dist (void){
  //Derived formula
  int ADCval = analogRead(back_sensor);
  float dist = 1/(0.0002*ADCval - 0.0051);   //20.9/ADCval *(5/1023.0);
  return dist; //in cm
}

//INITIALISING
void orientation (void) {
  float leftFrontDist, leftBackDist, dist; 
  //Initialise to garbage values
  float error = 6.2;
  float angle = 3.14/2;

  int ccwTurn, strafeRight, frontControl, rearControl, tinit;
  int ccwGain = 1500;
  int strafeGain = 50;
  int t = 0;
  
  //Keep going if the angle is more than 2 degrees or the distance is out by 0.2 cm and if the values are changing
  while(((abs(angle) > 0.0349) || (abs(error) > 0.2)) && (t < 2000)){
    leftFrontDist = left_front_dist();
    leftBackDist = left_back_dist();
    
    //distance from the wall
    dist = (leftFrontDist + leftBackDist)/2;
    //approximate sin theta to theta
    angle = (leftFrontDist - leftBackDist)/separationDist;

    //ensure that the left sensors are reading the distance values from the left wall before controlling distance from the wall
    if ((angle < 3.14/8) && (angle > -3.14/8)) {
      //Reference angle is 0, therefore angle is the error
      //control signal for turning CCW
      ccwTurn = angle * ccwGain;
      //6.2 = 15 - 8.8 (dist from centre of robot to sensor)
      error = 6.2 - dist;
      //Control signal for strafing right
      strafeRight = error * strafeGain ;
    } else{
      ccwTurn = 500;
      strafeRight = 0;
    }

    //Control signal for front motors
    frontControl = constrain((-ccwTurn + strafeRight), -500, 500);

    //Control signal for rear motors
    rearControl = constrain((-ccwTurn - strafeRight), -500, 500);
    
    left_font_motor.writeMicroseconds(1500 + frontControl);
    left_rear_motor.writeMicroseconds(1500 + rearControl);
    right_rear_motor.writeMicroseconds(1500 + rearControl);
    right_font_motor.writeMicroseconds(1500 + frontControl);
    
    SerialCom -> print("angle = ");
    SerialCom -> println(angle);
    SerialCom -> print(", error = ");
    SerialCom -> println(error);
    SerialCom -> println(t);
    
    //If the control signals are low for too long, start the timer to quit so that the motors don't burn out
    if ((abs(strafeRight) < 50) && (abs(ccwTurn) < 50)){
      t = t + millis() - tinit;
    } else {
      t = 0;
    }
  }
}

void turn_90_gyro(void){
  /*Need to check positives and negatives, and adjust... also need to do a units check to make sure values aren't garbage*/
  //Take CW to be positive
  float currentAngle = 0;
  float error = 90;
  float rotationalGain = 26.2; //Gain of 1500/180*pi, same gain as the orientation code
  float angleChange;
  int tinit, t, motorControl;
  
  // convert the 0-1023 signal to 0-5v
  while (abs(error) > 2){ //add more exit conditions if need be
    tinit = millis();
    gyroRate = (analogRead(sensorPin) * gyroSupplyVoltage) / 1023; 
    // find the voltage offset the value of voltage when gyro is zero (still)
    gyroRate -= (gyroZeroVoltage / 1023 * 5); 
    // read out voltage divided the gyro sensitivity to calculate the angular velocity
    float angularVelocity = gyroRate / gyroSensitivity;  // Ensure that +ve velocity is taken in the CW direction
    // if the angular velocity is less than the threshold, ignore it
    if ((angularVelocity >= rotationThreshold) || (angularVelocity <= -rotationThreshold)) { // we are running a loop in T. one second will run (1000/T).
      angleChange = angularVelocity / (1000 / T);
      
      currentAngle += angleChange; //check sign
    }  
    
    // keep the angle between 0-360 - for P control, don't
    /*if (currentAngle < 0)    {
      currentAngle += 360;
    }  else if (currentAngle > 359) {
      currentAngle -= 360;
    } */
    
    error = 90 - currentAngle;

    motorControl = error * rotationalGain;
    motorControl = constrain(motorControl, -500, 500);

    left_font_motor.writeMicroseconds(1500 + motorControl);
    left_rear_motor.writeMicroseconds(1500 + motorControl);
    right_rear_motor.writeMicroseconds(1500 + motorControl);
    right_font_motor.writeMicroseconds(1500 + motorControl);

    //may not be necessary, CHECK
    t = millis() - tinit;

    SerialCom -> print ("error: ");
    SerialCom -> println (error);
    SerialCom -> print ("time: ");
    SerialCom -> println(t);

    delay (T - t);
  }
}

void turn_90 (void){
  float leftFrontDist,leftBackDist, dist, angle;
  float error = 20; //initialise error to garbage value

  int t, cwTurn, strafeRight, frontControl, rearControl;
  int cwGain = -1500; //negative as the turn is in the opposite direction of ccwGain
  int strafeGain = 50;
  int separationDist = 19;
  int backDist = back_dist();

  int tinit = millis();

  //turns for an amount to get sensors past the corner - replace with gyro code
  while((backDist > 30) || (backDist < 2) || t < 1000){ //Adjust t limit or do clever things with backDist
    backDist = back_dist();
    t = millis() - tinit;
    left_font_motor.writeMicroseconds(2000);
    left_rear_motor.writeMicroseconds(2000);
    right_rear_motor.writeMicroseconds(2000);
    right_font_motor.writeMicroseconds(2000);
  }
  
  while(((abs(angle) > 0.0349) || (abs(error) > 0.2)) && (t < 2000)){
    leftFrontDist = left_front_dist();
    leftBackDist = left_back_dist();
    
    //distance from the wall
    dist = (leftFrontDist + leftBackDist)/2;
    //approximate sin theta to theta
    angle = (leftFrontDist - leftBackDist)/separationDist;

    if ((angle < 3.14/8) && (angle > -3.14/8)) {
      //Reference angle is 0, control signal for turning CW
      cwTurn = angle * cwGain;
      //6.2 = 15 - 8.8 (dist from centre of robot to sensor)
      error = 6.2 - dist;
      
      //Control signal for strafing right
      strafeRight = error * strafeGain ;
      strafeRight = (strafeRight > 500) ? 500 : strafeRight;
    } else{
      cwTurn = 500;
      strafeRight = 0;
    }

    //Control signal for front motors
    frontControl = (cwTurn + strafeRight > 500) ? 500 : cwTurn + strafeRight;
    frontControl = (cwTurn + strafeRight < -500) ? -500 : frontControl;

    //Control signal for rear motors
    rearControl = (cwTurn - strafeRight > 500) ? 500 : -cwTurn - strafeRight;
    rearControl = (cwTurn - strafeRight < -500) ? -500 : rearControl;
    
    left_font_motor.writeMicroseconds(1500 + frontControl);
    left_rear_motor.writeMicroseconds(1500 + rearControl);
    right_rear_motor.writeMicroseconds(1500 + rearControl);
    right_font_motor.writeMicroseconds(1500 + frontControl);
    
    SerialCom -> print("angle = ");
    SerialCom -> println(angle);
    SerialCom -> print(", error = ");
    SerialCom -> println(error);
    SerialCom -> println(t);
    
    //If the control signals are low for too long, start the timer to quit so that the motors don't burn out
    if ((abs(strafeRight) < 50) && (abs(cwTurn) < 50)){
      t = t + millis() - tinit;
    } else {
      t = 0;
    }
  }
}

int yawController (void){
  /*Returns the control signal for a yaw controller, can be integrated within the go straight for more performance
  We should actually be able to superimpose all 3 control signals together, I'll discuss with Andrew more in the lab*/
  int ccwGain = 1500; //same as initialising gain
  int ccwTurn;
  float leftFrontDist, leftBackDist, angle;
  
  leftFrontDist = left_front_dist();
  leftBackDist = left_back_dist();

  //approximate sin theta to theta
  angle = (leftFrontDist - leftBackDist)/separationDist;
  ccwTurn = angle * ccwGain;
  ccwTurn = constrain(ccwTurn, -500, 500);
  return ccwTurn;
}

//RUNNING
void forward(void){

}


void enable_motors()
{
  left_font_motor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}

void stop() //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void cw(){
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void left(){
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}
