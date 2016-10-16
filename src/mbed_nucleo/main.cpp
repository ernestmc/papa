
#include "mbed.h"

/*Pin map
#define INA1     D2
#define INB1     D4
#define EN1DIAG1 D6
#define CS1      A0
#define INA2     D7
#define INB2     D8
#define EN2DIAG2 D12
#define CS2      A1
*/

DigitalOut led2(LED2);

//DualVNH5019AccelMotorShield motors;

DigitalOut   INA_1(D1);
DigitalOut   INB_1(D4);
DigitalInOut ENDIAG_1(D6);
AnalogIn     CS_1(A0);
PwmOut       PWM1(D9);

/*DigitalOut   INA_2(D7);
DigitalOut   INB_2(D8);
DigitalInOut ENDIAG_2(D12);
AnalogIn     CS_2(A1);
PwmOut       PWM2(D10);*/



/*VNH5019Accel m1 = motors(1);
VNH5019Accel m2 = motors(2);

void enable(void) {
  m1.enable();
  m2.enable();
}

void brake(void) {
  m1.brake(1);
  m2.brake(1);
}

void set_speed(float speed) {
  m1.speed(speed);
  m2.speed(speed);
}

void clear_fault(VNH5019Accel m) {
  if (m.is_fault()) {
    m.clear_fault();
  }
}

void clear_faults() {
  clear_fault(m1);
  clear_fault(m2);
}*/

void init(){
  ENDIAG_1.input();
  ENDIAG_1.mode(PullUp);
  PWM1.period(0.00025);   // 4 kHz (valid 0 - 20 kHz)
  PWM1.write(0);
  INA_1 = 0;
  INB_1 = 0;

  /*ENDIAG_2.input();
  ENDIAG_2.mode(PullUp);
  PWM2.period(0.00025);   // 4 kHz (valid 0 - 20 kHz)
  PWM2.write(0);
  INA_2 = 0;
  INB_2 = 0;*/
}


void speed(float Speed)
{
   bool Reverse = 0;

   if (Speed < 0)
   {
     Speed = -Speed;  // Make speed a positive quantity
     Reverse = 1;  // Preserve the direction
   }

   // clamp the speed at maximum
   if (Speed > 1.0)
      Speed = 1.0;

   if (Speed == 0.0)
   {
       INA_1 = 0;
       INB_1 = 0;
       PWM1 = 0;
    }
    else
    {
      INA_1 = !Reverse;
      INB_1 = Reverse;
      PWM1 = Speed;
    }
}



int main() {

  /*brake();
  clear_faults();
  enable();

  float step = 0.01;
  float speed = -1;*/
  init();

  while(true) {
    PWM1 = 1;
    INA_1 = 0;
    INB_1 = 1;

    led2 = !led2;
    wait(0.5);
    //speed(0.5);


    /*while (speed <= 1.0) {
      set_speed(speed);
      speed += step;
    }
    while (speed >= -1.0) {
      set_speed(speed);
      speed -= step;
    }*/
  }
}
