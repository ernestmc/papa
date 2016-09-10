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

/*  VNH2SP30 pin definitions
 xxx[0] controls '1' outputs
 xxx[1] controls '2' outputs */
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
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

  if ( cmd_msg.angular.z == 0 && cmd_msg.linear.x == 0 ) {
    motorGo(0, BRAKEGND, 0);
    motorGo(1, BRAKEGND, 0);
  } else {
    if ( cmd_msg.angular.z < -0 ) {  // atras
      motorGo(0, CCW, 1023);
      motorGo(1, CCW, 1023);
    } else if ( cmd_msg.angular.z > 0 ) { // adelante
      motorGo(0, CW, 1023);
      motorGo(1, CW, 1023);
    } else if ( cmd_msg.linear.x < 0.0 ) { // derecha
      motorGo(0, CW, 1023);
      motorGo(1, CCW, 1023);
    } else if ( cmd_msg.linear.x > 0.0 ) {   // izquierda
      motorGo(0, CCW, 1023);
      motorGo(1, CW, 1023);
    }
  }
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
  if ( millis() >= range_time ) {

    pub_ping.publish(&ping_msg);
    range_time =  millis() + 50;
  }
  nh.spinOnce();

}
