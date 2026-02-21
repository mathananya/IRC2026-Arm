#include "cytrons.h"
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>

#define NUM_MOTORS 8

// <----------------- ARM PINS --------------->
#define WRIST1_PWM 5
#define WRIST1_DIR 44
#define WRIST1_ENCA 37
#define WRIST1_ENCB 35

#define WRIST2_PWM 13
#define WRIST2_DIR 30
#define WRIST2_ENCA 39
#define WRIST2_ENCB 41

#define GRIPPY_PWM 6
#define GRIPPY_DIR 42
#define GRIPPY_ENCA 45
#define GRIPPY_ENCB 43

#define LA1_PWM 11
#define LA1_DIR 49
#define LA1_POTEN A7

#define LA2_PWM 10
#define LA2_DIR 48
#define LA2_POTEN A6

#define LA1_UPPER_LIMIT 600
#define LA1_LOWER_LIMIT 240

#define LA2_UPPER_LIMIT 510
#define LA2_LOWER_LIMIT 100

bool LA1_U_LIMIT_FLAG = false;
bool LA1_L_LIMIT_FLAG = false;
bool LA2_U_LIMIT_FLAG = false;
bool LA2_L_LIMIT_FLAG = false;

#define LA_LIMIT_MARGIN 10

// < -------------- MICRO ROS DEFINE --------------->
// rcl_publisher_t publisher1;
rcl_publisher_t publisher2;
// rcl_subscription_t subscriber1;
rcl_subscription_t subscriber2;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator = rcl_get_default_allocator();
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }

void error_loop() {
  while (1) {
    delay(100);
  }
}

// <----------------- ARM MOTORS --------------->
MDD10A wrist1_motor(WRIST1_PWM, WRIST1_DIR, WRIST1_ENCA, WRIST1_ENCB);
MDD10A wrist2_motor(WRIST2_PWM, WRIST2_DIR, WRIST2_ENCA, WRIST2_ENCB);
MDD10A grippy_motor(GRIPPY_PWM, GRIPPY_DIR, GRIPPY_ENCA, GRIPPY_ENCB);
MDD10A LA1(LA1_PWM, LA1_DIR, 0, 0);
MDD10A LA2(LA2_PWM, LA2_DIR, 0, 0);

// <---------------- ARM VARIABLE SETUP -------->

int arm_posi[3] = { 0, 0, 0 };
const char *arm_map[3] = {
  "Wrist 1",
  "Wrist 2",
  "Grippy"
};
int LA1_enc;
int LA2_enc;
std_msgs__msg__Int32MultiArray arm_encoder_msg;
std_msgs__msg__Int32MultiArray arm_pwm_msg;

volatile uint32_t last_message_time_arm = 0;
volatile int arm_pwm_values[5] = { 0 };

// < -------------- SUBSCRIBER AND PUBLISHER CALLBACKS ------------- >

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Copy encoder positions to message

    arm_encoder_msg.data.data[0] = LA1_enc;
    arm_encoder_msg.data.data[1] = LA2_enc;
    arm_encoder_msg.data.data[2] = arm_posi[0];
    arm_encoder_msg.data.data[3] = arm_posi[1];
    arm_encoder_msg.data.data[4] = arm_posi[2];

    RCSOFTCHECK(rcl_publish(&publisher2, &arm_encoder_msg, NULL));
  }
}


void subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *msg =
    (const std_msgs__msg__Int32MultiArray *)msgin;

  // Copy received ARM PWM values
  for (int i = 0; i < 5; i++) {
    arm_pwm_values[i] = msg->data.data[i];
  }

  last_message_time_arm = millis();
}


// < ----------- SETUP -------------------->

void setup() {
  set_microros_transports();

  // Setup arm encoders
  attachInterrupt(WRIST1_ENCA, readEncoderArm1, CHANGE);
  attachInterrupt(WRIST2_ENCA, readEncoderArm2, CHANGE);
  attachInterrupt(GRIPPY_ENCA, readEncoderArm3, CHANGE);

  delay(2000);
  allocator = rcl_get_default_allocator();

  // Micro-ROS initialization
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "multi_motor_node", "", &support));


  // Allocate memory for multiarray messages
  arm_encoder_msg.data.capacity = 5;
  arm_encoder_msg.data.size = 5;
  arm_encoder_msg.data.data = (int32_t *)malloc(5 * sizeof(int32_t));

  arm_pwm_msg.data.capacity = 5;
  arm_pwm_msg.data.size = 5;
  arm_pwm_msg.data.data = (int32_t *)malloc(5 * sizeof(int32_t));

  // Create publisher and subscriber

  RCCHECK(rclc_publisher_init_default(
    &publisher2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "arm_encoder_data"));  // Create publisher

  RCCHECK(rclc_subscription_init_default(
    &subscriber2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "arm_pwm_commands"));  // Create subscriber

  // Timer setup
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Executor setup
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber2, &arm_pwm_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  uint32_t current_time = millis();

  //<----------------- ARM TEST --------------->

  if (current_time - last_message_time_arm > 2000) {
    //only running loop till 4 beacuse gripper has no feedback as of now
    for (int i = 0; i < 5; i++) {
      arm_pwm_values[i] = 0;
    }
  }

  LA1_enc = analogRead(LA1_POTEN);
  LA2_enc = analogRead(LA2_POTEN);

  if (LA1_enc > (LA1_LOWER_LIMIT - LA_LIMIT_MARGIN) && LA1_enc < (LA1_UPPER_LIMIT + LA_LIMIT_MARGIN)) {
    LA1.run(arm_pwm_values[0]);
  } else if (LA1_enc > (LA1_UPPER_LIMIT + LA_LIMIT_MARGIN)) {
    LA1.run(max(0, arm_pwm_values[0]));
  } else if (LA1_enc < (LA1_LOWER_LIMIT - LA_LIMIT_MARGIN)) {
    LA1.run(min(0, arm_pwm_values[0]));
  }


  if (LA2_enc > (LA2_LOWER_LIMIT - LA_LIMIT_MARGIN) && LA2_enc < (LA2_UPPER_LIMIT + LA_LIMIT_MARGIN)) {
    LA2.run(arm_pwm_values[1]);
  } else if (LA2_enc > (LA2_UPPER_LIMIT + LA_LIMIT_MARGIN)) {
    LA2.run(min(0, arm_pwm_values[1]));
  } else if (LA2_enc < (LA2_LOWER_LIMIT - LA_LIMIT_MARGIN)) {
    LA2.run(max(0, arm_pwm_values[1]));
  }

  wrist1_motor.run(arm_pwm_values[2]);
  wrist2_motor.run(arm_pwm_values[3]);
  grippy_motor.run(arm_pwm_values[4]);

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}

void readEncoderArm1() {
  static int lastENCA = LOW;
  int currentENCA = digitalRead(WRIST1_ENCA);
  int currentENCB = digitalRead(WRIST1_ENCB);

  // Determine direction based on ENCA and ENCB
  if (currentENCA != lastENCA) {
    if (currentENCA == HIGH) {
      if (currentENCB == LOW) {
        arm_posi[0]++;
      } else {
        arm_posi[0]--;
      }
    } else {
      if (currentENCB == LOW) {
        arm_posi[0]--;
      } else {
        arm_posi[0]++;
      }
    }
    lastENCA = currentENCA;
  }
}

void readEncoderArm2() {
  static int lastENCA = LOW;
  int currentENCA = digitalRead(WRIST2_ENCA);
  int currentENCB = digitalRead(WRIST2_ENCB);

  // Determine direction based on ENCA and ENCB
  if (currentENCA != lastENCA) {
    if (currentENCA == HIGH) {
      if (currentENCB == LOW) {
        arm_posi[1]++;
      } else {
        arm_posi[1]--;
      }
    } else {
      if (currentENCB == LOW) {
        arm_posi[1]--;
      } else {
        arm_posi[1]++;
      }
    }
    lastENCA = currentENCA;
  }
}

void readEncoderArm3() {
  static int lastENCA = LOW;
  int currentENCA = digitalRead(GRIPPY_ENCA);
  int currentENCB = digitalRead(GRIPPY_ENCB);

  // Determine direction based on ENCA and ENCB
  if (currentENCA != lastENCA) {
    if (currentENCA == HIGH) {
      if (currentENCB == LOW) {
        arm_posi[2]++;
      } else {
        arm_posi[2]--;
      }
    } else {
      if (currentENCB == LOW) {
        arm_posi[2]--;
      } else {
        arm_posi[2]++;
      }
    }
    lastENCA = currentENCA;
  }
}