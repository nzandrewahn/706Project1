#include <Servo.h>
//#include <functions.h>

enum STATE
{
  INITIALISING,
  RUNNING,
  STOPPED
};


// Define motions states
enum MOTION
{
  FORWARD,
  BACKWARD,
  LEFT_TURN,
  RIGHT_TURN,
  LEFT_ARC,
  RIGHT_ARC,
  BACKWARD_LEFT_TURN,
};


void setup()
{
    Serial.begin(9600);
}

void loop()
{

    //Put your main code here, to run repeatedly:
    static STATE machine_state = INITIALISING; // start from the sate INITIALIING
    switch (machine_state)
    {
    case INITIALISING:
        machine_state = initialising();
        break;
    case RUNNING:
        machine_state = running();
        break;
    case STOPPED:
        machine_state = stopped();
        break;
    }
}



STATE initialising(void){
  return RUNNING;
}


STATE running(void){
  return STOPPED;
}

STATE stopped(void){
  return STOPPED;
}
