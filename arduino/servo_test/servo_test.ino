#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <Servo.h>

Servo l1h;
Servo l1k;
// Servo l2h;
// Servo l2k;
// Servo l3h;
// Servo l3k;
// Servo l4h;
// Servo l4k;
ros::NodeHandle nh;

int value[8] = {45.0, -88.0, 49.0, -82.0, 45.0, -88.0, 49.0, -82.0};

void messageCb(const std_msgs::Float64MultiArray &toggle_msg)
{
  digitalWrite(LED_BUILTIN, HIGH - digitalRead(LED_BUILTIN)); // blink the led
  for (int i = 0; i < 8; i++)
  {
    value[i] = toggle_msg.data[i];
  }
}

ros::Subscriber<std_msgs::Float64MultiArray> sub("js_real", &messageCb);

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  l1h.attach(9);
  l1k.attach(8);
}

void loop()
{
  l1h.write(value[0]);
  l1k.write(-value[1]);  //some angles kept always negative or always positive for simulation
  nh.spinOnce();
  delay(1);
}
