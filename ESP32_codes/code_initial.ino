#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>


#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>


// ----- Hardware Pins -----
#define LEFT_ENCODER_A_PIN  21
#define LEFT_ENCODER_B_PIN  47
#define RIGHT_ENCODER_A_PIN 48
#define RIGHT_ENCODER_B_PIN 45
#define ENA 14
#define IN1 13
#define IN2 12
#define IN3 11
#define IN4 10
#define ENB 9

// ----- Encoder & RPM -----
#define INTERVAL 100
#define PULSES_PER_REVOLUTION 560
volatile int left_ticks = 0;
volatile int right_ticks = 0;


// ----- ROS 2 Entities -----
rclc_support_t support;
rcl_node_t node;
rcl_timer_t rpm_timer;
rclc_executor_t executor;
rcl_allocator_t allocator;


rcl_publisher_t left_tick_pub, right_tick_pub, left_rpm_pub, right_rpm_pub;
std_msgs__msg__Int32 left_tick_msg, right_tick_msg;
std_msgs__msg__Float32 left_rpm_msg, right_rpm_msg;


rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;


// ----- Agent Management -----
enum states {
 WAITING_AGENT,
 AGENT_AVAILABLE,
 AGENT_CONNECTED,
 AGENT_DISCONNECTED
} state;


// ----- Macros -----
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) return false; }
#define EXECUTE_EVERY_N_MS(MS, X) do { \
 static volatile int64_t init = -1; \
 if (init == -1) { init = uxr_millis(); } \
 if (uxr_millis() - init > MS) { X; init = uxr_millis(); } \
} while (0)


// ----- Encoder ISRs -----
void IRAM_ATTR leftEncoderISR() { left_ticks++; }
void IRAM_ATTR rightEncoderISR() { right_ticks++; }


// ----- Motor Control -----
void cmd_vel_callback(const void * msgin) {
 const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;


 float linear = msg->linear.x;
 float angular = msg->angular.z;


 const float max_speed = 255.0;
 const float max_linear = 0.2;
 const float max_angular = 1.5;


 int base_speed = (int)(fabs(linear) / max_linear * max_speed);
 base_speed = constrain(base_speed, 0, 255);


 int turn_offset = (int)(fabs(angular) / max_angular * max_speed);
 turn_offset = constrain(turn_offset, 0, 255);


 int left_speed = base_speed;
 int right_speed = base_speed;


 if (angular > 0) {
   left_speed -= turn_offset;
   right_speed += turn_offset;
 } else if (angular < 0) {
   left_speed += turn_offset;
   right_speed -= turn_offset;
 }


 left_speed = constrain(left_speed, 0, 255);
 right_speed = constrain(right_speed, 0, 255);


 if (linear > 0) {
   digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
   digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
 } else if (linear < 0) {
   digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
   digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
 } else {
   if (angular > 0) {
     digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
     digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
   } else if (angular < 0) {
     digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
     digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
   } else {
     digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
     digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
   }
 }


 analogWrite(ENA, left_speed);
 analogWrite(ENB, right_speed);
}


// ----- Timer Callback -----
void rpm_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
 (void) last_call_time;
 if (timer != NULL) {
   left_tick_msg.data = left_ticks;
   right_tick_msg.data = right_ticks;


   float left_rpm = (left_ticks * (60000.0 / INTERVAL)) / PULSES_PER_REVOLUTION;
   float right_rpm = (right_ticks * (60000.0 / INTERVAL)) / PULSES_PER_REVOLUTION;
   left_rpm_msg.data = left_rpm;
   right_rpm_msg.data = right_rpm;


   rcl_publish(&left_tick_pub, &left_tick_msg, NULL);
   rcl_publish(&right_tick_pub, &right_tick_msg, NULL);
   rcl_publish(&left_rpm_pub, &left_rpm_msg, NULL);
   rcl_publish(&right_rpm_pub, &right_rpm_msg, NULL);


   left_ticks = 0;
   right_ticks = 0;
 }
}


// ----- Create & Destroy Entities -----
bool create_entities() {
 allocator = rcl_get_default_allocator();
 RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
 RCCHECK(rclc_node_init_default(&node, "encoder_node", "", &support));


 RCCHECK(rclc_publisher_init_default(&left_tick_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "left_ticks"));
 RCCHECK(rclc_publisher_init_default(&right_tick_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "right_ticks"));
 RCCHECK(rclc_publisher_init_default(&left_rpm_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "left_rpm"));
 RCCHECK(rclc_publisher_init_default(&right_rpm_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "right_rpm"));


 RCCHECK(rclc_subscription_init_default(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));


 const unsigned int timer_timeout = INTERVAL;
 RCCHECK(rclc_timer_init_default(&rpm_timer, &support, RCL_MS_TO_NS(timer_timeout), rpm_timer_callback));


 executor = rclc_executor_get_zero_initialized_executor();
 RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
 RCCHECK(rclc_executor_add_timer(&executor, &rpm_timer));
 RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));


 return true;
}


void destroy_entities() {
 rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
 (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);


 rcl_publisher_fini(&left_tick_pub, &node);
 rcl_publisher_fini(&right_tick_pub, &node);
 rcl_publisher_fini(&left_rpm_pub, &node);
 rcl_publisher_fini(&right_rpm_pub, &node);
 rcl_subscription_fini(&cmd_vel_sub, &node);
 rcl_timer_fini(&rpm_timer);
 rclc_executor_fini(&executor);
 rcl_node_fini(&node);
 rclc_support_fini(&support);
}


// ----- Arduino Setup -----
void setup() {
 set_microros_transports();



 pinMode(LEFT_ENCODER_A_PIN, INPUT_PULLUP);
 pinMode(LEFT_ENCODER_B_PIN, INPUT_PULLUP);
 pinMode(RIGHT_ENCODER_A_PIN, INPUT_PULLUP);
 pinMode(RIGHT_ENCODER_B_PIN, INPUT_PULLUP);
 attachInterrupt(LEFT_ENCODER_A_PIN, leftEncoderISR, RISING);
 attachInterrupt(RIGHT_ENCODER_A_PIN, rightEncoderISR, RISING);


 pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
 pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
 pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);


 state = WAITING_AGENT;
}


// ----- Arduino Loop -----
void loop() {
 switch (state) {
   case WAITING_AGENT:
     EXECUTE_EVERY_N_MS(500,
       state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
     );
     break;


   case AGENT_AVAILABLE:
     state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
     if (state == WAITING_AGENT) destroy_entities();
     break;


   case AGENT_CONNECTED:
     EXECUTE_EVERY_N_MS(200,
       state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
     );
     if (state == AGENT_CONNECTED)
       rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
     break;


   case AGENT_DISCONNECTED:
     destroy_entities();
     state = WAITING_AGENT;
     break;
 }

}
