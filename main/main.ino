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

  Author: CloutGods
*/

#include <Servo.h> //Need for Servo pulse output

//#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

#define WALL_DISTANCE (15)
#define FRONT_DISTANCE_LIMIT (15)
#define ANTICLOCKWISE (1000)
#define CLOCKWISE (2000)
#define SERVO_STOP_VALUE (1500)
#define WINDOW_SIZE (300)
#define YAW_THRESHHOLD (500)

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

int left_back_dist();
int left_front_dist();
int right_dist();
int front_dist();
void LeftDistanceController();

//Tuning Parameters
int Kp = 50;
int Kd = 0;
int Ki = 0;

//Serial Pointer
HardwareSerial *SerialCom;

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
  return RUNNING;
}

STATE running()
{

  // Check if turning count is higher than 4 if yes then return

  // Read Sensor values

  // Store values
  int frontDist = front_dist();
  int cornerCount = 0;

  // Decide which way to go based on new value vs old value, so the difference between the old and new value is the error and we exit when front is less than 15cm
  while (true)
  {

    int avgDistance = (left_front_dist() + left_back_dist()) / 2;
    int TOLERANCE = 3;
    int error = WALL_DISTANCE - avgDistance;
    int front_offset = constrain(error*Kp,0,500);
    int rear_offset = constrain(error*Kp,0,500);

    Serial.print("Error: ");
    Serial.println(error);
  
    while (error > TOLERANCE || error < TOLERANCE)
    {
      
      
      left_front_motor.writeMicroseconds(SERVO_STOP_VALUE + front_offset);
      right_front_motor.writeMicroseconds(SERVO_STOP_VALUE + front_offset);
      left_rear_motor.writeMicroseconds(SERVO_STOP_VALUE - rear_offset);
      right_rear_motor.writeMicroseconds(SERVO_STOP_VALUE - rear_offset);

      avgDistance = (left_front_dist() + left_back_dist())/2;
      error = WALL_DISTANCE - avgDistance;
      front_offset = constrain(error*Kp,-500,500);
      rear_offset = constrain(error*Kp,-500,500);
    }
//    GoForwards();
//    frontDist = front_dist();

  }

  // Run turning function

  // Increment no of corners

  stop();

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
  return STOPPED;
}

/////////////////////////////////////////////// Helper Functions

////////// Reading Sensor

int left_front_dist()
{
  int Dis2 = 1 / (0.0005 * analogRead(A7) - 0.0101);
  return Dis2;
}

int left_back_dist()
{
  int Dis1 = 1 / (0.0005 * analogRead(A4) - 0.0078);
  return Dis1;
}

int front_dist()
{
  int Dis4 = (1 / (0.0002 * analogRead(A15) - 0.0051) - 2.5);
  return Dis4;
}

int back_dist()
{
  int Dis3 = (1 / (0.0002 * analogRead(A11) - 0.0068) - 2.5);
  return Dis3;
}

int read_yaw()
{
  return 1;
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
void stop() //Stop
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

// Motor Commands
void GoForwards(void)
{
  left_front_motor.writeMicroseconds(CLOCKWISE);
  right_front_motor.writeMicroseconds(ANTICLOCKWISE);
  left_rear_motor.writeMicroseconds(CLOCKWISE);
  right_rear_motor.writeMicroseconds(ANTICLOCKWISE);
}

void GoBackwards(void)
{
  left_front_motor.writeMicroseconds(ANTICLOCKWISE);
  right_front_motor.writeMicroseconds(ANTICLOCKWISE);
  left_rear_motor.writeMicroseconds(ANTICLOCKWISE);
  right_rear_motor.writeMicroseconds(ANTICLOCKWISE);
}

void StrafeRight(void)
{
  left_front_motor.writeMicroseconds(ANTICLOCKWISE);
  right_front_motor.writeMicroseconds(ANTICLOCKWISE);
  left_rear_motor.writeMicroseconds(CLOCKWISE);
  right_rear_motor.writeMicroseconds(CLOCKWISE);
}

void StrafeLeft(void)
{
  left_front_motor.writeMicroseconds(CLOCKWISE);
  right_front_motor.writeMicroseconds(CLOCKWISE);
  left_rear_motor.writeMicroseconds(ANTICLOCKWISE);
  right_rear_motor.writeMicroseconds(ANTICLOCKWISE);
}

void TurnRight(void)
{
  left_front_motor.writeMicroseconds(CLOCKWISE);
  right_front_motor.writeMicroseconds(ANTICLOCKWISE);
  left_rear_motor.writeMicroseconds(CLOCKWISE);
  right_rear_motor.writeMicroseconds(ANTICLOCKWISE);
}

void TurnLeft(void)
{
  left_front_motor.writeMicroseconds(ANTICLOCKWISE);
  right_front_motor.writeMicroseconds(CLOCKWISE);
  left_rear_motor.writeMicroseconds(ANTICLOCKWISE);
  right_rear_motor.writeMicroseconds(CLOCKWISE);
}
