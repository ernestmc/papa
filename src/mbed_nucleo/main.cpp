#include "VNH5019Accel.h"
#include "mbed.h"

//Pin map
#define INA1     D2
#define INB1     D4
#define EN1DIAG1 D6
#define CS1      A0
#define INA2     D7
#define INB2     D8
#define EN2DIAG2 D12
#define CS2      A1

DualVNH5019AccelMotorShield motors;
VNH5019Accel m1 = motors(1);
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
}

int main() {

  brake();
  clear_faults();
  enable();

  float step = 0.01;
  float speed = -1;

  while(true) {
    while (speed <= 1.0) {
      set_speed(speed);
      speed += step;
    }
    while (speed >= -1.0) {
      set_speed(speed);
      speed -= step;
    }
  }
}
