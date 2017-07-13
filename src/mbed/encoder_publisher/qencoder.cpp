#include "mbed.h"
#include "QEI.h"
#include <ros.h>
#include <std_msgs/Int32.h>
#include <rosserial_mbed/wheel_count.h>

#define PULSES_REVOLUTION 36

QEI rightEncoder(p7, p8, NC, PULSES_REVOLUTION);
QEI leftEncoder(p29, p30, NC, PULSES_REVOLUTION);
ros::NodeHandle  nh;

rosserial_mbed::wheel_count wheel_msg;
ros::Publisher wheels("wheel_count", &wheel_msg);
DigitalOut led = LED1;

int main() {
    nh.initNode();
    nh.advertise(wheels);
    while(1) {
        led = !led;
        wheel_msg.right_wheel = rightEncoder.getPulses();
        wheel_msg.left_wheel = leftEncoder.getPulses();
        wheels.publish(&wheel_msg);
        nh.spinOnce();
        wait_ms(100);
    }
}
