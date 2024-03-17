
/*
TO PUBLISH DATA TO pwm_duty_cycle TOPIC FROM TERMINAL: 

ros2 topic pub /pwm_duty_cycle --once std_msgs/msg/Float32 "data: 100"
*/

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>  

rcl_publisher_t pot_publisher_handle;
rcl_publisher_t vol_publisher_handle;
rcl_subscription_t pwm_dc_subscriber_handle;

std_msgs__msg__Int32 raw_pot_msg;
std_msgs__msg__Float32 voltage_msg;
std_msgs__msg__Float32 duty_cycle_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_timer_t timer1_handle;
rcl_timer_t timer2_handle;

#define LED_PIN 13 // FOR DEBUGGING
#define ADC_PIN 36
#define PWM_PIN 15

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

const uint16_t pwm_frequency = 5000;
const uint8_t pwm_channel = 0;
const uint8_t pwm_resolution = 8;
float pwm_duty_cycle = 0;

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

//CALLBACKS
//------------------------------------------------------------------
void timer1_callback(rcl_timer_t * timer, int64_t last_call_time){  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    raw_pot_msg.data = analogRead(ADC_PIN);
    voltage_msg.data = raw_pot_msg.data * (3.3/4095);
  }
}

void timer2_callback(rcl_timer_t * timer, int64_t last_call_time){  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&vol_publisher_handle, &voltage_msg, NULL));
    RCSOFTCHECK(rcl_publish(&pot_publisher_handle, &raw_pot_msg, NULL));
  }
}

void subscription_callback(const void * msgin){
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  if(msg->data > 100.0){
    pwm_duty_cycle  = 100.0;
  }else if(msg->data < 0.0){
    pwm_duty_cycle  = 0.0;
  }else{
    pwm_duty_cycle = msg->data;
  }
  pwm_duty_cycle = pwm_duty_cycle * (255.0/100.0);
  ledcWrite(pwm_channel, pwm_duty_cycle);
}
//------------------------------------------------------------------


void setup() {

  set_microros_transports();
  allocator = rcl_get_default_allocator();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(1000);

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  //-------------------------------------------------------------------------------------

  //PWM SETUP
  ledcSetup(pwm_channel, pwm_frequency, pwm_resolution);
  ledcAttachPin(PWM_PIN, pwm_channel);
  
  //CREATE NODE
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32", "", &support));

  //CREATE PUBLISHERS
  RCCHECK(rclc_publisher_init_default(
  &pot_publisher_handle,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  "raw_pot"));

  RCCHECK(rclc_publisher_init_default(
  &vol_publisher_handle,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
  "voltage"));

  //CREATE SUBSCRIBERS
  RCCHECK(rclc_subscription_init_default(
  &pwm_dc_subscriber_handle,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
  "pwm_duty_cycle"));

  //CREATE TIMERS (FOR TIMER CALLBACKS)
  const unsigned int timer_1 = 100;
  RCCHECK(rclc_timer_init_default(
  &timer1_handle,
  &support,
  RCL_MS_TO_NS(timer_1),
  timer1_callback));

  const unsigned int timer_2 = 10;
  RCCHECK(rclc_timer_init_default(
  &timer2_handle,
  &support,
  RCL_MS_TO_NS(timer_2),
  timer2_callback));

  //-------------------------------------------------------------------------------------s

  //CREATE EXECUTOR
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer1_handle));
  RCCHECK(rclc_executor_add_timer(&executor, &timer2_handle));
  RCCHECK(rclc_executor_add_subscription(&executor, &pwm_dc_subscriber_handle, &duty_cycle_msg, &subscription_callback, ON_NEW_DATA));

  //INITIALIZE MSG VARIABLES 
  raw_pot_msg.data = 0;
  voltage_msg.data = 0;
  duty_cycle_msg.data = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
