#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>

#include <stdint.h>



#define LEFT_ENCODER_A_PIN  21  //   BLUE
#define LEFT_ENCODER_B_PIN  47  //   VIOLET
#define RIGHT_ENCODER_A_PIN 48 //   BLUE
#define RIGHT_ENCODER_B_PIN 45  //     VIOLET
#define INTERVAL 100  // Time interval in milliseconds
#define PULSES_PER_REVOLUTION 560  // PPR


#define ENA 14
#define IN1 13
#define IN2 12
#define IN3 11
#define IN4 10
#define ENB 9




volatile int32_t left_ticks = 0;
volatile int32_t right_ticks = 0;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t left_tick_pub;
rcl_publisher_t right_tick_pub;
// rcl_publisher_t left_rpm_pub;
// rcl_publisher_t right_rpm_pub;
std_msgs__msg__Int32 left_tick_msg;
std_msgs__msg__Int32 right_tick_msg;
// std_msgs__msg__Float32 left_rpm_msg;
// std_msgs__msg__Float32 right_rpm_msg;


rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;




void IRAM_ATTR leftEncoderISR() {
  left_ticks++;
}




void IRAM_ATTR rightEncoderISR() {
  right_ticks++;
}



void cmd_vel_callback(const void * msgin) {
 const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;


 float linear = msg->linear.x;    // m/s
 float angular = msg->angular.z;  // rad/s


 // Constants (tune as per your setup)
 const float max_speed = 255.0;  // PWM range
 const float max_linear = 0.2;   // m/s (max expected linear speed)
 const float max_angular = 1.5;  // rad/s (max expected turn rate)


 // Compute base speed from linear velocity
 int base_speed = (int)(fabs(linear) / max_linear * max_speed);
 if (base_speed > 255) base_speed = 255;


 // Compute turning offset from angular velocity
 int turn_offset = (int)(fabs(angular) / max_angular * max_speed);
 if (turn_offset > 255) turn_offset = 255;


 // Determine direction and motor speeds
 int left_speed = base_speed;
 int right_speed = base_speed;


 if (angular > 0) { // Turning Left
   left_speed -= turn_offset;
   right_speed += turn_offset;
 } else if (angular < 0) { // Turning Right
   left_speed += turn_offset;
   right_speed -= turn_offset;
 }


 // Clamp values
 left_speed = constrain(left_speed, 0, 255);
 right_speed = constrain(right_speed, 0, 255);


 // Apply direction based on linear.x
 if (linear > 0) {
   // Forward
   digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
   digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
 } else if (linear < 0) {
   // Backward
   digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
   digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
 } else {
   // Stop or Rotate in place
   if (angular > 0) {
     // Rotate Left
     digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
     digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
   } else if (angular < 0) {
     // Rotate Right
     digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
     digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
   } else {
     // Full Stop
     digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
     digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
   }
 }


 // Set motor speeds
 analogWrite(ENA, left_speed);
 analogWrite(ENB, right_speed);
}


void setup() {
  set_microros_transports();
  pinMode(LEFT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(LEFT_ENCODER_A_PIN, leftEncoderISR, RISING);
  attachInterrupt(RIGHT_ENCODER_A_PIN, rightEncoderISR, RISING);




  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "encoder_publisher", "", &support);
  rclc_publisher_init_default(&left_tick_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "left_ticks");
  rclc_publisher_init_default(&right_tick_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "right_ticks");
  // rclc_publisher_init_default(&left_rpm_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "left_rpm");
  // rclc_publisher_init_default(&right_rpm_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "right_rpm");
  rclc_executor_init(&executor, &support.context, 2, &allocator);


  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);


 rclc_subscription_init_default(
   &cmd_vel_sub,
   &node,
   ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
   "cmd_vel"
 );


 rclc_executor_add_subscription(
   &executor, &cmd_vel_sub, &cmd_vel_msg,
   &cmd_vel_callback, ON_NEW_DATA
 );


 
}




void loop() {
  static unsigned long last_time = 0;
  if (millis() - last_time >= INTERVAL) {
      last_time = millis();
      left_tick_msg.data = left_ticks;
      right_tick_msg.data = right_ticks;




      // float left_rpm = (left_ticks * (60000.0 / INTERVAL)) / PULSES_PER_REVOLUTION;
      // float right_rpm = (right_ticks * (60000.0 / INTERVAL)) / PULSES_PER_REVOLUTION;
      // left_rpm_msg.data = left_rpm;
      // right_rpm_msg.data = right_rpm;




      rcl_publish(&left_tick_pub, &left_tick_msg, NULL);
      rcl_publish(&right_tick_pub, &right_tick_msg, NULL);
      // rcl_publish(&left_rpm_pub, &left_rpm_msg, NULL);
      // rcl_publish(&right_rpm_pub, &right_rpm_msg, NULL);


  }
  //delay(10);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
}
