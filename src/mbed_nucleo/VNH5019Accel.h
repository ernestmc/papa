// VNH5015 driver with acceleration control
// A drop-in replacement for the basic VNH5019

#if !defined(VNH5019ACCEL_H)
#define VNH5019ACCEL_H

#include "VNH5019.h"
#include <Ticker.h>

// The maximum acceleration, in fractions of full speed per second
float const VNH5019MaxAccel = 4;

// Tick period in microseconds for updating the speed
int const VNH5019Tick_us = 20000;

float const VNH5019ChangePerTick = VNH5019MaxAccel * VNH5019Tick_us / 1000000;

// allow braking to come on faster
float const VNH5019BrakeChangePerTick = VNH5019ChangePerTick*10;

class VNH5019Accel
{
    public:
        VNH5019Accel(PinName INA_, PinName INB_, PinName ENDIAG_, PinName CS_, PinName PWM_)
            : Driver(INA_, INB_, ENDIAG_, CS_, PWM_),
              CurrentSpeed(0.0),
              CurrentBrake(0.0),
              RequestedSpeed(0.0),
              RequestedBrake(0.0)
        {
            timer.attach_us(this, &VNH5019Accel::Interrupt, VNH5019Tick_us);
        }

        // set motor speed from -1.0 to +1.0
        void speed(float Speed)
        {
            if (Speed < -1.0)
                Speed = 1.0;
            else if (Speed > 1.0)
                Speed = 1.0;
            RequestedSpeed = Speed;
        }

        // stop (no current to the motors)
        void stop()
        {
            RequestedSpeed = 0.0;
        }

        // Brake, with strength 0..1
        void brake(float Brake)
        {
            if (Brake < 0.0)
                Brake = 0;
            else if (Brake > 1.0)
                Brake = 1.0;
            RequestedBrake = Brake;
            RequestedSpeed = 0.0;
        }

        // returns the current through the motor, in mA
        float get_current_mA()
        {
            return Driver.get_current_mA();
        }

        // returns true if there has been a fault
        bool is_fault()
        {
            return Driver.is_fault();
        }

        // Clears the fault condition
        // PRECONDITION: is_fault()
        void clear_fault()
        {
            timer.detach();
            Driver.clear_fault();
            timer.attach_us(this, &VNH5019Accel::Interrupt, VNH5019Tick_us);
        }

        // disable the motor, and set outputs to zero.  This is a low power mode.
        void disable()
        {
            timer.detach();
            Driver.disable();
        }

        // enable the motor.
        void enable()
        {
            timer.detach();
            Driver.enable();
            timer.attach_us(this, &VNH5019Accel::Interrupt, VNH5019Tick_us);
        }

        // set the PWM period of oscillation in seconds
        void set_pwm_period(float p)
        {
            Driver.set_pwm_period(p);
        }

    private:
        VNH5019 Driver;
        Ticker timer;
        float CurrentSpeed;  // this is only ever accessed from the ISR, so no need for volatile
        float CurrentBrake;
        volatile float RequestedSpeed;
        volatile float RequestedBrake;
        void Interrupt();
};

// Helper class for the Pololu dual VNH5019 motor shield.
// The default constructor uses the default arduino pins.
// The motors can be accessed either by .m1 or .m2, or by operator()(i) where i is 1 or 2.
class DualVNH5019AccelMotorShield
{
    public:
        // default pin selection
        DualVNH5019AccelMotorShield(); // Default pin selection.

                              // User-defined pin selection.
        DualVNH5019AccelMotorShield(PinName INA1_, PinName INB1_, PinName ENDIAG1_, PinName CS1_, PinName PWM1_,
                                    PinName INA2_, PinName INB2_, PinName ENDIAG2_, PinName CS2_, PinName PWM2_)
         : m1(INA1_, INB1_, ENDIAG1_, CS1_, PWM1_),
           m2(INA2_, INB2_, ENDIAG2_, CS2_, PWM2_)
        {
        }

        // returns the given motor object, 1 or 2.
        VNH5019Accel& operator()(int m)
        {
            return m == 1 ? m1 : m2;
        }

      VNH5019Accel m1;
      VNH5019Accel m2;
};

#endif
