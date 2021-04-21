/*
  MechEng 706 Base Code
  This code provides basic movement and sensor reading for the MechEng 706 Mecanum Wheel Robot Project
  Hardware:
    Arduino Mega2560 https://www.arduino.cc/en/Guide/ArduinoMega2560
    MPU-9250 https://www.sparkfun.com/products/13762
    Ultrasonic Sensor - HC-SR04 https://www.sparkfun.com/products/13959
    Infrared Proximity Sensor - Sharp https://www.sparkfun.com/products/242
    Infrared Proximity Sensor Short Range - Sharp https://www.sparkfun.com/products/12728
    Servo - Generic (Sub-Micro Size) https://www.sparkfun.com/products/9065
    Vex Motor Controller 29 https://www.vexrobotics.com/276-2193.html
    Vex Motors https://www.vexrobotics.com/motors.html
    Turnigy nano-tech 2200mah 2S https://hobbyking.com/en_us/turnigy-nano-tech-2200mah-2s-25-50c-lipo-pack.html
  Date: 11/11/2016
  Author: Logan Stuart
  Modified: 15/02/2018
  Author: Logan Stuart
*/

#include <Servo.h> //Need for Servo pulse output

//#define NO_READ_GYRO  //Uncomment if GYRO is not attached.
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

#define WALL_DISTANCE (7.6) //7.45
#define FRONT_DISTANCE_LIMIT (6)
#define ANTICLOCKWISE (1000)
#define CLOCKWISE (2000)
#define STOP (1500)
#define SERVO_STOP_VALUE (1500)
#define WINDOW_SIZE (300)
#define YAW_THRESHHOLD (500)
#define YAW_TOLERANCE (3)

//State machine states
enum STATE
{
  INITIALISING,
  RUNNING,
  STOPPED
};

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

Servo left_front_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_front_motor; // create servo object to control Vex Motor Controller 29
Servo turret_motor;

int separationDist = 19; // Distance between the two IR sensors
int cornerCount = 0;     // Number of corners currently turned
bool complete = false;   // Checking if the obstacle has been completed

//Gyro Variables
int T = 100;
int gyroPin = 12;
float gyroSupplyVoltage = 5;
float gyroZeroVoltage = 509;
float gyroSensitivity = 0.007;
float rotationThreshold = 1.5;
float gyroRate = 0;

//Serial Pointer
HardwareSerial *SerialCom;

//Function Declarations
float front_dist();

void setup(void)
{
  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");

  delay(1000); //settling time but no really needed
}

void loop(void) //main loop
{
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state)
  {
  case INITIALISING:
    machine_state = initialising();
    break;
  case RUNNING: //Lipo Battery Volage OK
    machine_state = running();
    break;
  case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
    machine_state = stopped();
    break;
  };
}

////////////////////////////// FSM STATES

STATE initialising()
{
  // Initialising, enable motors
  SerialCom->println("INITIALISING....");
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");

  if (!is_battery_voltage_OK())
  {
    SerialCom->println("Battery is not okay...");
    return INITIALISING;
  }

  return RUNNING;
}

STATE running()
{
  //From the previous command the robot should be oriented
  orientation();
  delay(300);
  goStraight();

  // Increment number of corners
  cornerCount++;

  // Check if turning count is higher than 4 if yes then return
  if (cornerCount >= 4)
  {
    stop();
    complete = true;
    return STOPPED;
  }

  // Run turning function and orient against the wall if the turning limit has not been reached
  turn_90_gyro();

  return RUNNING;
}

//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped()
{
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500)
  { //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");
    //Only check whether the robot can revert to running state if it has not completed its loop
    if (!complete)
    {
#ifndef NO_BATTERY_V_OK
      //500ms timed if statement to check lipo and output speed settings
      if (is_battery_voltage_OK())
      {
        SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
        SerialCom->println(counter_lipo_voltage_ok);
        counter_lipo_voltage_ok++;
        if (counter_lipo_voltage_ok > 10)
        { //Making sure lipo voltage is stable
          counter_lipo_voltage_ok = 0;
          enable_motors();
          SerialCom->println("Lipo OK returning to RUN STATE");
          return RUNNING;
        }
      }
      else
      {
        counter_lipo_voltage_ok = 0;
      }
#endif
    }
  }
  return STOPPED;
}

/////////////////////////////////////////////// Helper Functions

////////// Reading Sensor

float left_front_dist()
{
  //Returns the distance read by the front left IR sensor
  float Dis2 = 1 / (0.0005 * analogRead(A7) - 0.0101);
  return Dis2;
}

float left_back_dist()
{
  //Returns the distance read by the back left IR sensor
  float Dis1 = 1 / (0.0005 * analogRead(A4) - 0.0078);
  return Dis1;
}

float front_dist()
{
  //Returns the distance read by the front IR sensor
  float Dis4 = (1 / (0.0002 * analogRead(A15) - 0.0051) - 2.5);
  return Dis4;
}

float back_dist()
{
  //Returns the distance read by the back IR sensor
  float Dis3 = (1 / (0.0002 * analogRead(A11) - 0.0068) - 2.5);
  return Dis3;
}

////////////////////////////// Reading Battery Voltage

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK()
{
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160)
  {
    previous_millis = millis();
    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    SerialCom->println("");
    Low_voltage_counter = 0;
    return true;
  }
  else
  {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overcharged!!!");
    else
    {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }
}
#endif

// Motor Functions
void disable_motors()
{
  left_front_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();   // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_front_motor.detach(); // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_front_motor.attach(left_front);   // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);     // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);   // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_front_motor.attach(right_front); // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop() // Stopping function
{
  left_front_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_front_motor.writeMicroseconds(1500);
}

/////////////////////////////// Led
void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000)
  {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void fast_flash_double_LED_builtin()
{
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis)
  {
    indexer++;
    if (indexer > 4)
    {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    }
    else
    {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

////////////////// Controllers
void goStraight(void)
{
  float leftFrontDist = left_front_dist();
  float leftBackDist = left_back_dist();

  float avgDistance = 0;    //Average distance from the left wall
  float left_error = 0;     //Error from the distance of the wall that was set
  float left_I_error = 0;   //Integral of the left error
  float left_I_gain = 0.04; //Left controller I gain
  int left_P_gain = 120;    //Left controller P gain
  int left_control;         //Control action for left controller

  float angle;             //The angle from being straight from the wall
  float ccw_I_error = 0;   //Angle controller integral error
  int ccwGain = 2000;      //Angle controller P gain, same as initialising gain
  float ccw_I_gain = 0.25; //Angle controller I gain
  float ccwTurn;           //Control action for angle controller

  float forward_error = 10; //Error from the desired distance at the front of the wall
  float forward_gain = -30; //Front controller P gain
  int forward_control = 0;  //Control action for front controller

  //Control actions for each motor
  int left_front_motor_control, right_front_motor_control, left_rear_motor_control, right_rear_motor_control;

  SerialCom->println("Entered goStraight Function");

  while (abs(forward_error) > 1)
  {
    //Get IR distances
    leftFrontDist = left_front_dist();
    leftBackDist = left_back_dist();

    //Calculate the distance away from the wall
    avgDistance = (leftFrontDist + leftBackDist) / 2;
    left_error = WALL_DISTANCE - avgDistance;

    //To prevent integrator wind up, only calculate the integral control effort if the error is less than 4 cm
    if (abs(left_error) < 4)
    {
      left_I_error += left_error;
    }

    //Calculate the control effort to move left
    left_control = constrain(int(left_error * left_P_gain + left_I_error * left_I_gain), -500, 500);

    //approximate sin theta to theta
    angle = (leftFrontDist - leftBackDist) / separationDist;
    ccw_I_error += angle;

    //Calculate the turning control effort and saturate it so that the turning and strafing control efforts do not exceed 500 when added
    ccwTurn = constrain(int(angle * ccwGain + ccw_I_error * ccw_I_gain), -500 + abs(left_control), 500 - abs(left_control));

    //Calculate the forward movement control effort and saturate it so that the forward, turning and strafing control efforts do not exceed 500 when added
    forward_error = FRONT_DISTANCE_LIMIT - front_dist();
    forward_control = constrain(forward_error * forward_gain, -400 + abs(left_control) + abs(ccwTurn), 400 - abs(left_control) - abs(ccwTurn));

    //Calculate each motor's control effort by superimposing each controller in the relevant direction
    left_front_motor_control = constrain(forward_control + left_control - ccwTurn, -500, 500);
    right_front_motor_control = constrain(-forward_control + left_control - ccwTurn, -500, 500);
    left_rear_motor_control = constrain(forward_control - left_control - ccwTurn, -500, 500);
    right_rear_motor_control = constrain(-forward_control - left_control - ccwTurn, -500, 500);

    //Send power to the motors
    left_front_motor.writeMicroseconds(SERVO_STOP_VALUE + left_front_motor_control);
    right_front_motor.writeMicroseconds(SERVO_STOP_VALUE + right_front_motor_control);
    left_rear_motor.writeMicroseconds(SERVO_STOP_VALUE + left_rear_motor_control);
    right_rear_motor.writeMicroseconds(SERVO_STOP_VALUE + right_rear_motor_control);

    Serial.print("left error: ");
    Serial.println(left_error);
    Serial.print("I error: ");
    Serial.println(left_I_error);
    Serial.print("CCW error: ");
    Serial.println(angle);
  }

  //Stop the motors when the front error is less than 1 cm
  left_front_motor.writeMicroseconds(SERVO_STOP_VALUE);
  right_front_motor.writeMicroseconds(SERVO_STOP_VALUE);
  left_rear_motor.writeMicroseconds(SERVO_STOP_VALUE);
  right_rear_motor.writeMicroseconds(SERVO_STOP_VALUE);
}

void orientation(void)
{
  float separationDist = 19;
  float leftFrontDist, leftBackDist, dist;
  float error = 6.2;
  float angle = 3.14 / 2;
  int ccwTurn, strafeRight, frontControl, rearControl, tinit;
  int ccwGain = 2000;
  int strafeGain = 120;
  int t = 0;

  while (((abs(angle) > 0.0349) || (abs(error) > 0.2)) && (t < 1000))
  {
    //Get IR distances
    leftFrontDist = left_front_dist();
    leftBackDist = left_back_dist();

    //Find the distance from the wall
    dist = (leftFrontDist + leftBackDist) / 2;

    //Approximate sin theta to theta
    angle = (leftFrontDist - leftBackDist) / separationDist;

    //Only use strafing controller if the angle is sufficiently small, otherwise turn CCW only
    if ((angle < 3.14 / 8) && (angle > -3.14 / 8))
    {
      //Reference angle is 0, control signal for turning CCW
      ccwTurn = constrain(int(angle * ccwGain), -500, 500);

      error = WALL_DISTANCE +0.2 - dist;
      //Control signal for strafing right, constrain so that when superimposed on ccwTurn, the combined signal will not exceed 500
      strafeRight = constrain(int(error * strafeGain), -500 + abs(ccwTurn), 500 - abs(ccwTurn));
    }
    else
    {
      ccwTurn = 500;
      strafeRight = 0;
    }

    //Superimpose the two control signals

    //Control signal for front motors
    frontControl = (-ccwTurn + strafeRight > 500) ? 500 : -ccwTurn + strafeRight;
    frontControl = (-ccwTurn + strafeRight < -500) ? -500 : frontControl;

    //Control signal for rear motors
    rearControl = (-ccwTurn - strafeRight > 500) ? 500 : -ccwTurn - strafeRight;
    rearControl = (-ccwTurn - strafeRight < -500) ? -500 : rearControl;

    left_front_motor.writeMicroseconds(1500 + frontControl);
    left_rear_motor.writeMicroseconds(1500 + rearControl);
    right_rear_motor.writeMicroseconds(1500 + rearControl);
    right_front_motor.writeMicroseconds(1500 + frontControl);

    //    SerialCom->print("angle = ");
    //    SerialCom->println(angle);
    //    SerialCom->print(", error = ");
    //    SerialCom->println(error);
    //    SerialCom->println(t);

    //If the control signals are low for too long, start the timer to quit so that the motors don't burn out
    if ((abs(strafeRight) < 50) && (abs(ccwTurn) < 50))
    {
      t = t + millis() - tinit;
    }
    else
    {
      t = 0;
    }
  }
  SerialCom->print("Orientation finished");
}

void turn_90_gyro(void)
{
  /*Need to check positives and negatives, and adjust... also need to do a units check to make sure values aren't garbage*/
  //Take CW to be positive
  float currentAngle = 0;
  float error = 90;
  float rotationalGain = 25; //26.2; //Gain of 1500/180*pi, same gain as the orientation code
  float angleChange, angularVelocity;
  int tinit, t, motorControl;

  //SerialCom ->println();
  // convert the 0-1023 signal to 0-5v
  while (abs(error) > 2)
  { //add more exit conditions if need be

    tinit = millis();
    gyroRate = (analogRead(gyroPin) * gyroSupplyVoltage) / 1023;
    // find the voltage offset the value of voltage when gyro is zero (still)
    gyroRate -= (gyroZeroVoltage / 1023 * 5);

    // read out voltage divided the gyro sensitivity to calculate the angular velocity
    angularVelocity = gyroRate / gyroSensitivity; // Ensure that +ve velocity is taken in the CW direction
    SerialCom->println(angularVelocity);

    // if the angular velocity is less than the threshold, ignore it
    if ((angularVelocity >= rotationThreshold) || (angularVelocity <= -rotationThreshold))
    { // we are running a loop in T. one second will run (1000/T).
      angleChange = angularVelocity / (1000 / T);

      currentAngle += angleChange; //check sign
    }

    //Calculating the error signal for the control loop
    error = 90 - currentAngle;

    motorControl = error * rotationalGain;
    motorControl = constrain(motorControl, -500, 500);

    // Sending control signals to the motors
    left_front_motor.writeMicroseconds(1500 + motorControl);
    left_rear_motor.writeMicroseconds(1500 + motorControl);
    right_rear_motor.writeMicroseconds(1500 + motorControl);
    right_front_motor.writeMicroseconds(1500 + motorControl);

    //may not be necessary, CHECK
    //delay(10);
    t = millis() - tinit;

    SerialCom->print("time: ");
    SerialCom->println(tinit);

    delay(T - t);
  }
  //Stop motors
  left_front_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_front_motor.writeMicroseconds(1500);
}
