#include "mbed.h"
#include <ros.h>
#include <std_msgs/Int32.h>
#include <rosserial_mbed/wheel_count.h>

#define PULSES_REVOLUTION 36

InterruptIn rightEncoder(p11);
InterruptIn leftEncoder(p30);
ros::NodeHandle  nh;

int rightPulses = 0;
int leftPulses = 0;
rosserial_mbed::wheel_count wheel_msg;
ros::Publisher wheels("wheel_count", &wheel_msg);
DigitalOut led = LED1;

void rightInterrupt() {
    rightPulses++; 
}

void leftInterrupt() {
    leftPulses++;  
}

int main() {
    nh.initNode();
    nh.advertise(wheels);
    rightEncoder.rise(&rightInterrupt);
    leftEncoder.rise(&leftInterrupt);
    while(1) {
        led = !led;
        wheel_msg.right_wheel = rightPulses;
        wheel_msg.left_wheel = leftPulses;
        wheels.publish(&wheel_msg);
        nh.spinOnce();
        wait_ms(100);
    }
}
