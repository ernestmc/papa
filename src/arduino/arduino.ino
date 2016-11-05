#define USE_USBCON

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>

#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100

#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1


/*  VNH2SP30 pin definitions
 xxx[0] controls '1' outputs
 xxx[1] controls '2' outputs */
int inApin[2] = {2, 7};  // INA: Clockwise input
int inBpin[2] = {4, 8}; // INB: Counter-clockwise input
int pwmpin[2] = {9, 10}; // PWM input
int cspin[2] = {2, 3}; // CS: Current sense ANALOG input
int enpin[2] = {0, 1}; // EN: Status of switches output (Analog pin)

int statpin = 13;


ros::NodeHandle nh;
std_msgs::String ping_msg;


void init_motor(void) {
  // Initialize digital pins as outputs
  for (int i=0; i<2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
}


/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.
 
 motor: this should be either 0 or 1, will selet which of the two
 motors to be controlled
 
 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND
 
 pwm: should be a value between ? and 1023, higher the number, the faster
 it'll go
 */
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <=4)
    {
      // Set inA[motor]
      if (direct <=1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct==0)||(direct==2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}

void messageCb( const geometry_msgs::Twist& cmd_msg) {
  float rightVel, leftVel;
  if (abs(cmd_msg.angular.z) < 0.05 && abs(cmd_msg.linear.x) < 0.05) {
    motorGo(RIGHT_MOTOR, BRAKEGND, 0);
    motorGo(LEFT_MOTOR, BRAKEGND, 0);
  } else {
      rightVel = (cmd_msg.linear.x + cmd_msg.angular.z / 2.0) * 100;
      leftVel = (cmd_msg.linear.x - cmd_msg.angular.z / 2.0) * 100;
      if (rightVel > 0)
        motorGo(RIGHT_MOTOR, CW, abs(rightVel));
       else
        motorGo(RIGHT_MOTOR, CCW, abs(rightVel));
      if (leftVel > 0)
        motorGo(LEFT_MOTOR, CW, abs(leftVel));
       else
        motorGo(LEFT_MOTOR, CCW, abs(leftVel));
    }
}

void test_adelante(void)
{
    motorGo(0, CW, 100);
    motorGo(1, CW, 100);
}

void test_atras(void)
{
    motorGo(0, CCW, 100);
    motorGo(1, CCW, 100);
}

void test_girar_derecha()
{
    motorGo(LEFT_MOTOR, CW, 100);
    motorGo(RIGHT_MOTOR, CCW, 100);  
}

void test_girar_izquierda()
{
    motorGo(LEFT_MOTOR, CCW, 100);
    motorGo(RIGHT_MOTOR, CW, 100);  
}

ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", messageCb );
ros::Publisher pub_ping( "/ping", &ping_msg);

void setup()
{
  init_motor();

  nh.initNode();
  nh.advertise(pub_ping);
  nh.subscribe(sub_vel);

  ping_msg.data = "todo esta bien";
  
}

long range_time;

void loop()
{
  //test_adelante();
  //test_girar_derecha();
  
  if ( millis() >= range_time ) {

    pub_ping.publish(&ping_msg);
    range_time =  millis() + 50;
  }
  nh.spinOnce();
}
