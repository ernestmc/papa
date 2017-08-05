#include "mbed.h"
#include <ros.h>
#include <std_msgs/Int32.h>
#include <rosserial_mbed/wheel_count.h>
#include <rosserial_mbed/papa_imu.h>
#include <rosserial_mbed/papa_sonar.h>
#include "MPU6050.h"
#include "HC_SR04_Ultrasonic_Library/ultrasonic.h"

#define PULSES_REVOLUTION 36

InterruptIn rightEncoder(p11);
InterruptIn leftEncoder(p30);
ros::NodeHandle  nh;

MPU6050 mpu6050;

int rightPulses = 0;
int leftPulses = 0;
int range1 = 0;
int range2 = 0;
rosserial_mbed::wheel_count wheel_msg;
rosserial_mbed::papa_imu imu_msg;
rosserial_mbed::papa_sonar sonar_msg;
ros::Publisher wheels("wheel_count", &wheel_msg);
ros::Publisher imu_publisher("papa_imu", &imu_msg);
ros::Publisher sonar_publisher("papa_sonar", &sonar_msg);
DigitalOut led(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);

// testing generator
DigitalOut test_pin(p18);
Ticker test_tic;

uint8_t imu_read_who_ami() {
    // Read the WHO_AM_I register, this is a good test of communication. it shpuld be 0x68
    return mpu6050.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
}    

void imu_init() {
    mpu6050.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
    wait(1);
 
    if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) 
    {
        mpu6050.resetMPU6050(); // Reset registers to default in preparation for device calibration
        mpu6050.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
        mpu6050.initMPU6050(); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    }
}

void test_interrupt() {
    test_pin = !test_pin;
}

void rightInterrupt() {
    rightPulses++;
    led2 = !led2;
}

void leftInterrupt() {
    leftPulses++;
    led3 = !led3;  
}

void publish_imu(void) {
    // If data ready bit set, all data registers have new data
    if(mpu6050.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt
        mpu6050.readAccelData(accelCount);  // Read the x/y/z adc values
        mpu6050.getAres();
    
        // Now we'll calculate the accleration value into actual g's
        ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
        ay = (float)accelCount[1]*aRes - accelBias[1];   
        az = (float)accelCount[2]*aRes - accelBias[2];  
    
        mpu6050.readGyroData(gyroCount);  // Read the x/y/z adc values
        mpu6050.getGres();
    
        // Calculate the gyro value into actual degrees per second
        gx = (float)gyroCount[0]*gRes; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
        gy = (float)gyroCount[1]*gRes; // - gyroBias[1];  
        gz = (float)gyroCount[2]*gRes; // - gyroBias[2];   
    
        imu_msg.accel_x = ax;
        imu_msg.accel_y = ay;
        imu_msg.accel_z = az;
        imu_msg.gyro_x = gx;
        imu_msg.gyro_y = gy;
        imu_msg.gyro_z = gz;
        imu_publisher.publish(&imu_msg);
    }
}

void range1_int(int distance)
{
    range1 = distance;
}

void range2_int(int distance)
{
    range2 = distance;
}

//trigPin, echoPin, float updateSpeed, float timeout
ultrasonic sonar1(p15, p16, .1, 1, &range1_int);
ultrasonic sonar2(p17, p18, .1, 1, &range2_int);

void publish_sonar() {
    sonar_msg.ranges[0] = range1;
    sonar_msg.ranges[1] = range2;
    sonar_publisher.publish(&sonar_msg);
}

int main() {
    nh.initNode();
    nh.advertise(wheels);
    nh.advertise(imu_publisher);
    nh.advertise(sonar_publisher);
    rightEncoder.rise(&rightInterrupt);
    leftEncoder.rise(&leftInterrupt);
    test_tic.attach(&test_interrupt, 2.0);
    
    sonar1.startUpdates();
    sonar2.startUpdates();
    imu_init();
    
    while(1) {
        led = !led;
        wheel_msg.right_wheel = rightPulses;
        wheel_msg.left_wheel = leftPulses;
                 
        wheels.publish(&wheel_msg);
        sonar1.checkDistance();
        sonar2.checkDistance();
        publish_sonar();
        publish_imu();

        nh.spinOnce();
        wait_ms(100);
    }
}
