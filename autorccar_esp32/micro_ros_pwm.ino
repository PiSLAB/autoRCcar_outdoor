#include <micro_ros_arduino.h>
//#include <Servo.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/vector3.h>  // [steer accel tbd]

#include "esp32-hal-cpu.h"


rcl_subscription_t subscriber;
geometry_msgs__msg__Vector3 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


#define Pin_GLED 25   // Green LED
#define Pin_RLED 26   // Red LED
#define Pin_YLED 32   // Yellow LED
#define Pin_steer 17  // PWM Port
#define Pin_accel 16  // PWM Port
#define Pin_empty 4   // PWM Port (empty)

int pwm_ch0 = 0;
int pwm_ch1 = 1;
int pwm_ch2 = 2;
int pwm_freq = 50;        // 50Hz
int pwm_resolution = 16;  // 16bit


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void servoWrite(int ch, int deg) {
  int duty = deg*18.2 + 3277;
  ledcWrite(ch, duty);
}

void error_loop(){
  while(1)
  {
    digitalWrite(Pin_RLED, !digitalRead(Pin_RLED));
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{  
  const geometry_msgs__msg__Vector3 * msg = (const geometry_msgs__msg__Vector3 *)msgin;
  
  int xa, xs, xm;

  xs = msg->x;
  xa = msg->y;
  xm = msg->z;
  
  if (xs >= 180)
    xs = 180;
  if (xs <= 0)
    xs = 0;
  if (xa >= 180)
    xa = 180;
  if (xa <= 0)
    xa = 0;
 
  servoWrite(pwm_ch0, xs);  // steer pwm 0-180
  servoWrite(pwm_ch1, xa);  // accel pwm 0(reverse) 90(static) 180(accel)
  //servoWrite(pwm_ch2, msg->z);

  if (xa < 90)  // backward
    digitalWrite(Pin_YLED, HIGH);
  else
    digitalWrite(Pin_YLED, LOW);

  if (xm > 0)  // operating 
    digitalWrite(Pin_GLED, HIGH);
  else
    digitalWrite(Pin_GLED, LOW);

}

void setup() {
  set_microros_transports();

  setCpuFrequencyMhz(240); // Set CPU clock to 80, 160, 240MHz
  
  pinMode(Pin_GLED, OUTPUT);  
  pinMode(Pin_RLED, OUTPUT);
  pinMode(Pin_YLED, OUTPUT);
  
  digitalWrite(Pin_GLED, LOW);
  digitalWrite(Pin_RLED, LOW);
  digitalWrite(Pin_YLED, LOW);  

  ledcSetup(pwm_ch0, pwm_freq, pwm_resolution);
  ledcSetup(pwm_ch1, pwm_freq, pwm_resolution);
  ledcSetup(pwm_ch2, pwm_freq, pwm_resolution);
  ledcAttachPin(Pin_steer, pwm_ch0);
  ledcAttachPin(Pin_accel, pwm_ch1);
  ledcAttachPin(Pin_empty, pwm_ch2);

  servoWrite(pwm_ch0, 90);
  servoWrite(pwm_ch1, 90);
  delay(2000);


  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "param_ctl"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);  
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
