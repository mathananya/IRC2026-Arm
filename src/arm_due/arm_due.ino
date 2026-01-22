#include "cytrons.h"
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>

#define NUM_MOTORS 8

// <----------------- DRIVE PINS --------------->
#define DRIVE1_PWM 11
#define DRIVE1_DIR 34
#define DRIVE1_ENCA 22
#define DRIVE1_ENCB 23

#define DRIVE2_PWM 10
#define DRIVE2_DIR 36
#define DRIVE2_ENCA 24
#define DRIVE2_ENCB 25

#define DRIVE3_PWM 9
#define DRIVE3_DIR 38
#define DRIVE3_ENCA 51
#define DRIVE3_ENCB 50

#define DRIVE4_PWM 8
#define DRIVE4_DIR 40
#define DRIVE4_ENCA 52
#define DRIVE4_ENCB 53

// <----------------- STEPPER PINS --------------->
#define STEPPER1_1 27
#define STEPPER1_2 29
#define STEPPER1_3 31
#define STEPPER1_4 33

#define STEPPER2_1 47
#define STEPPER2_2 49
#define STEPPER2_3 46
#define STEPPER2_4 48

#define STEPPER3_1 14
#define STEPPER3_2 15
#define STEPPER3_3 16
#define STEPPER3_4 17

#define STEPPER4_1 2
#define STEPPER4_2 3
#define STEPPER4_3 20
#define STEPPER4_4 21

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

#define LA_LIMIT_MARGIN 35

// < -------------- MICRO ROS DEFINE --------------->
rcl_publisher_t publisher1;
rcl_publisher_t publisher2;
rcl_subscription_t subscriber1;
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

// <----------------- DRIVE MOTORS --------------->
MDD10A drive1_motor(DRIVE1_PWM, DRIVE1_DIR, DRIVE1_ENCA, DRIVE1_ENCB);  // Left-back
MDD10A drive2_motor(DRIVE2_PWM, DRIVE2_DIR, DRIVE2_ENCA, DRIVE2_ENCB);  // Left-front
MDD10A drive3_motor(DRIVE3_PWM, DRIVE3_DIR, DRIVE3_ENCA, DRIVE3_ENCB);  // Right-back
MDD10A drive4_motor(DRIVE4_PWM, DRIVE4_DIR, DRIVE4_ENCA, DRIVE4_ENCB);  // Right-front

MDD10A drive_motors[4] = { drive1_motor, drive2_motor, drive3_motor, drive4_motor };
int drive_posi[4] = { 0, 0, 0, 0 };

// <----------------- STEPPER MOTORS --------------->
MDD3A stepper1(STEPPER1_1, STEPPER1_2, STEPPER1_3, STEPPER1_4);  // Left-back
MDD3A stepper2(STEPPER2_1, STEPPER2_2, STEPPER2_3, STEPPER2_4);  // Left-front
MDD3A stepper3(STEPPER3_1, STEPPER3_2, STEPPER3_3, STEPPER3_4);  // Right-back
MDD3A stepper4(STEPPER4_1, STEPPER4_2, STEPPER4_3, STEPPER4_4);  // Right-front

MDD3A steppers[4] = { stepper1, stepper2, stepper3, stepper4 };

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

// <---------------- DRIVE VARIABLE SETUP -------->

std_msgs__msg__Int32MultiArray encoder_msg;
std_msgs__msg__Int32MultiArray pwm_msg;

volatile uint32_t last_message_time = 0;
int pwm_values[NUM_MOTORS] = { 0 };
int prev[4] = { 0, 0, 0, 0 };

// < -------------- SUBSCRIBER AND PUBLISHER CALLBACKS ------------- >

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Copy encoder positions to message
    for (int i = 0; i < 4; i++) {
      encoder_msg.data.data[i] = drive_posi[i];
    }

    arm_encoder_msg.data.data[0] = LA1_enc;
    arm_encoder_msg.data.data[1] = LA2_enc;
    arm_encoder_msg.data.data[2] = arm_posi[0];
    arm_encoder_msg.data.data[3] = arm_posi[1];
    arm_encoder_msg.data.data[4] = arm_posi[2];

    RCSOFTCHECK(rcl_publish(&publisher2, &arm_encoder_msg, NULL));
    RCSOFTCHECK(rcl_publish(&publisher1, &encoder_msg, NULL));
  }
}

void subscription_callback1(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *msg =
    (const std_msgs__msg__Int32MultiArray *)msgin;

  // Copy received PWM values
  for (int i = 0; i < NUM_MOTORS; i++) {
    pwm_values[i] = msg->data.data[i];
  }

  last_message_time = millis();
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

  // Setup drive encoders
  attachInterrupt(DRIVE1_ENCA, readEncoderDrive1, CHANGE);
  attachInterrupt(DRIVE2_ENCA, readEncoderDrive2, CHANGE);
  attachInterrupt(DRIVE3_ENCA, readEncoderDrive3, CHANGE);
  attachInterrupt(DRIVE4_ENCA, readEncoderDrive4, CHANGE);

  // Setup arm encoders
  attachInterrupt(WRIST1_ENCA, readEncoderArm1, CHANGE);
  attachInterrupt(WRIST2_ENCA, readEncoderArm2, CHANGE);
  attachInterrupt(GRIPPY_ENCA, readEncoderArm3, CHANGE);

  // Setup STEPPER 3 pins
  pinMode(STEPPER1_1, OUTPUT);
  pinMode(STEPPER1_2, OUTPUT);
  pinMode(STEPPER1_3, OUTPUT);
  pinMode(STEPPER1_4, OUTPUT);

  pinMode(STEPPER2_1, OUTPUT);
  pinMode(STEPPER2_2, OUTPUT);
  pinMode(STEPPER2_3, OUTPUT);
  pinMode(STEPPER2_4, OUTPUT);

  pinMode(STEPPER3_1, OUTPUT);
  pinMode(STEPPER3_2, OUTPUT);
  pinMode(STEPPER3_3, OUTPUT);
  pinMode(STEPPER3_4, OUTPUT);

  pinMode(STEPPER4_1, OUTPUT);
  pinMode(STEPPER4_2, OUTPUT);
  pinMode(STEPPER4_3, OUTPUT);
  pinMode(STEPPER4_4, OUTPUT);

  delay(2000);
  allocator = rcl_get_default_allocator();

  // Micro-ROS initialization
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "multi_motor_node", "", &support));

  // Allocate memory for multiarray messages
  encoder_msg.data.capacity = 4;
  encoder_msg.data.size = 4;
  encoder_msg.data.data = (int32_t *)malloc(4 * sizeof(int32_t));

  pwm_msg.data.capacity = NUM_MOTORS;
  pwm_msg.data.size = NUM_MOTORS;
  pwm_msg.data.data = (int32_t *)malloc(NUM_MOTORS * sizeof(int32_t));

  // Allocate memory for multiarray messages
  arm_encoder_msg.data.capacity = 5;
  arm_encoder_msg.data.size = 5;
  arm_encoder_msg.data.data = (int32_t *)malloc(5 * sizeof(int32_t));

  arm_pwm_msg.data.capacity = 5;
  arm_pwm_msg.data.size = 5;
  arm_pwm_msg.data.data = (int32_t *)malloc(5 * sizeof(int32_t));

  // Create publisher and subscriber
  RCCHECK(rclc_publisher_init_default(
    &publisher1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "encoder_data"));

  RCCHECK(rclc_publisher_init_default(
    &publisher2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "arm_encoder_data"));  // Create publisher

  RCCHECK(rclc_subscription_init_default(
    &subscriber1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "pwm_command"));

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
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber1, &pwm_msg, &subscription_callback1, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber2, &arm_pwm_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  // //<----------------- DRIVE TEST --------------->
  uint32_t current_time = millis();
  if (current_time - last_message_time > 2000) {
    for (int i = 0; i < 4; i++) {
      drive_motors[i].run(0);
    }
  }

  for (int i = 0; i < 4; i++) {
    drive_motors[i].run(pwm_values[i]);
  }

  // <----------------- STEPPERS TEST --------------->

  stepper1.update();
  stepper2.update();
  stepper3.update();
  stepper4.update();

  if (!stepper1.getIsRunning() && pwm_values[4] != prev[0]) {
    stepper1.run(1, pwm_values[4]);
    prev[0] = pwm_values[4];
  }
  if (!stepper2.getIsRunning() && pwm_values[5] != prev[1]) {
    stepper2.run(1, pwm_values[5]);
    prev[1] = pwm_values[5];
  }
  if (!stepper3.getIsRunning() && pwm_values[6] != prev[2]) {
    stepper3.run(1, pwm_values[6]);
    prev[2] = pwm_values[6];
  }
  if (!stepper4.getIsRunning() && pwm_values[7] != prev[3]) {
    stepper4.run(1, pwm_values[7]);
    prev[3] = pwm_values[7];
  }

  //<----------------- ARM TEST --------------->

  if (current_time - last_message_time_arm > 2000) {
    //only running loop till 4 beacuse gripper has no feedback as of now
    for (int i = 0; i < 5; i++) {
      arm_pwm_values[i] = 0;
    }
  }

  LA1_enc = analogRead(LA1_POTEN);
  LA2_enc = analogRead(LA2_POTEN);
  // if (LA1_enc > (LA1_UPPER_LIMIT + LA_LIMIT_MARGIN)) {
  //   LA1_U_LIMIT_FLAG = false;
  // }
  // if (LA1_enc < (LA1_LOWER_LIMIT - LA_LIMIT_MARGIN)) {
  //   LA1_L_LIMIT_FLAG = false;
  // }

  if (LA1_enc > (LA1_LOWER_LIMIT - 10) && LA1_enc < (LA1_UPPER_LIMIT + 10)) {
    LA1.run(arm_pwm_values[0]);
  } else if (LA1_enc > (LA1_UPPER_LIMIT + 10)) {
    LA1.run(max(0, arm_pwm_values[0]));
    // LA1_U_LIMIT_FLAG = true;
  } else if (LA1_enc < (LA1_LOWER_LIMIT - 10)) {
    LA1.run(min(0, arm_pwm_values[0]));
    // LA1_L_LIMIT_FLAG = true;
  }

  // if (LA2_enc > (LA2_UPPER_LIMIT + LA_LIMIT_MARGIN)) {
  //   LA2_U_LIMIT_FLAG = false;
  // }
  // if (LA2_enc < (LA2_LOWER_LIMIT - LA_LIMIT_MARGIN)) {
  //   LA2_L_LIMIT_FLAG = false;
  // }

  if (LA2_enc > (LA2_LOWER_LIMIT - 10) && LA2_enc < (LA2_UPPER_LIMIT + 10)) {
    LA2.run(arm_pwm_values[1]);
  } else if (LA2_enc > (LA2_UPPER_LIMIT + 10)) {
    LA2.run(min(0, arm_pwm_values[1]));
    // LA2_U_LIMIT_FLAG = true;
  } else if (LA2_enc < (LA2_LOWER_LIMIT - 10)) {
    LA2.run(max(0, arm_pwm_values[1]));
    // LA2_L_LIMIT_FLAG = true;
  }

  wrist1_motor.run(arm_pwm_values[2]);
  wrist2_motor.run(arm_pwm_values[3]);
  grippy_motor.run(arm_pwm_values[4]);

  if (!stepper1.getIsRunning() && !stepper2.getIsRunning() && !stepper3.getIsRunning() && !stepper4.getIsRunning()) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  }
}

void readEncoderDrive1() {
  static int lastENCA = LOW;
  int currentENCA = digitalRead(DRIVE1_ENCA);
  int currentENCB = digitalRead(DRIVE1_ENCB);

  // Determine direction based on ENCA and ENCB
  if (currentENCA != lastENCA) {
    if (currentENCA == HIGH) {
      if (currentENCB == LOW) {
        drive_posi[0]++;
      } else {
        drive_posi[0]--;
      }
    } else {
      if (currentENCB == LOW) {
        drive_posi[0]--;
      } else {
        drive_posi[0]++;
      }
    }
    lastENCA = currentENCA;
  }
}

void readEncoderDrive2() {
  static int lastENCA = LOW;
  int currentENCA = digitalRead(DRIVE2_ENCA);
  int currentENCB = digitalRead(DRIVE2_ENCB);

  // Determine direction based on ENCA and ENCB
  if (currentENCA != lastENCA) {
    if (currentENCA == HIGH) {
      if (currentENCB == LOW) {
        drive_posi[1]++;
      } else {
        drive_posi[1]--;
      }
    } else {
      if (currentENCB == LOW) {
        drive_posi[1]--;
      } else {
        drive_posi[1]++;
      }
    }
    lastENCA = currentENCA;
  }
}

void readEncoderDrive3() {
  static int lastENCA = LOW;
  int currentENCA = digitalRead(DRIVE3_ENCA);
  int currentENCB = digitalRead(DRIVE3_ENCB);

  // Determine direction based on ENCA and ENCB
  if (currentENCA != lastENCA) {
    if (currentENCA == HIGH) {
      if (currentENCB == LOW) {
        drive_posi[2]++;
      } else {
        drive_posi[2]--;
      }
    } else {
      if (currentENCB == LOW) {
        drive_posi[2]--;
      } else {
        drive_posi[2]++;
      }
    }
    lastENCA = currentENCA;
  }
}

void readEncoderDrive4() {
  static int lastENCA = LOW;
  int currentENCA = digitalRead(DRIVE4_ENCA);
  int currentENCB = digitalRead(DRIVE4_ENCB);

  // Determine direction based on ENCA and ENCB
  if (currentENCA != lastENCA) {
    if (currentENCA == HIGH) {
      if (currentENCB == LOW) {
        drive_posi[3]++;
      } else {
        drive_posi[3]--;
      }
    } else {
      if (currentENCB == LOW) {
        drive_posi[3]--;
      } else {
        drive_posi[3]++;
      }
    }
    lastENCA = currentENCA;
  }
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