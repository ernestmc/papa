#include "VNH5019Accel.h"

void VNH5019Accel::Interrupt()
{
    float MyRequestedSpeed = RequestedSpeed;
    if (CurrentSpeed != MyRequestedSpeed)
    {
        if (fabs(CurrentSpeed-MyRequestedSpeed) < VNH5019ChangePerTick)
            CurrentSpeed = MyRequestedSpeed;
        else if (MyRequestedSpeed > CurrentSpeed)
            CurrentSpeed += VNH5019ChangePerTick;
        else
            CurrentSpeed -= VNH5019ChangePerTick;
        Driver.speed(CurrentSpeed);
    }
    else
    {
        float MyRequestedBrake = RequestedBrake;
        if (CurrentBrake != MyRequestedBrake)
        {
            if (fabs(CurrentBrake-MyRequestedBrake) < VNH5019BrakeChangePerTick)
                CurrentBrake = MyRequestedBrake;
            else if (MyRequestedBrake > CurrentBrake)
                CurrentBrake += VNH5019BrakeChangePerTick;
          else
                CurrentBrake -= VNH5019BrakeChangePerTick;
            Driver.brake(CurrentBrake);
        }
    }
}

DualVNH5019AccelMotorShield::DualVNH5019AccelMotorShield()
: m1(D1, D4, D6, A0, D9),
  m2(D7, D8, D12, A1, D10){
}
