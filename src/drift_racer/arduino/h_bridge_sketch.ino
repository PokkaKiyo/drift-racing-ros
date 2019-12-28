#include <ros.h>
#include <geometry_msgs/Twist.h>

#define PWM_PIN 9
#define INPUT1 7
#define INPUT2 6

ros::NodeHandle nh;

unsigned long last_cmd_time = 0;

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  double brake_flag = cmd_vel_msg.linear.z;
  if (brake_flag != 0) {
    // set the speed to 0
    analogWrite(PWM_PIN, 0);
  }
  else
  {
    double desired_throttle = cmd_vel_msg.linear.x; // [0.0, 1.0]
    // set the duty cycle
    int pwm_value = desired_throttle * 255.0;
    analogWrite(PWM_PIN, pwm_value);
  }
  last_cmd_time = millis();
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel_final", &cmdVelCallback);

void setup()
{
  pinMode(PWM_PIN, OUTPUT);
  pinMode(INPUT1, OUTPUT);
  pinMode(INPUT2, OUTPUT);
  digitalWrite(INPUT1, LOW);
  digitalWrite(INPUT2, HIGH);
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);

  // If longer than 4s since last command, stop the throttle
  // For safety reasons if something disconnects
  if (millis() - last_cmd_time > 4000) {
    analogWrite(PWM_PIN, 0);
  }
}
