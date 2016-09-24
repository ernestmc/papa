#include "mbed.h"

#define INA1 D7
#define INA2 D4
#define INB1 D8
#define INB2 D9
#define PWM1 D5
#define PWM2 D6
#define CS1 A2
#define CS2 A3
#define EN1 A0
#define EN2 A1


//DigitalOut en(PA_7);
DigitalOut inA(PA_5);
DigitalOut inB(PA_6);
PwmOut pwm1(PA_8);

int main() {
    float val = 0;
    
    //en = 1;
    inA = 1;
    inB = 0;
    float inc = 0.01;
       
    while(1)
    {
        val += inc;
        if (val > 1 || val < 0) inc = -inc;
        pwm1 = val;
        wait_ms(100);
    }
}

