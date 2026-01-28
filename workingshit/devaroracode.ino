#include "cytrons.h"
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h> 


const int PIN_SEND_VP = 36; 
const int PIN_SEND_VN = 37; 
const int PIN_FEEDBACK = 30; 


const int FEEDBACK_IGNORE_TIME = 5000;      
const unsigned long SAFETY_TIMEOUT = 40000; 
const int POST_TASK_DELAY = 5000; 

bool command_active = false;         
unsigned long command_start_time = 0;


// Feedback State Helpers
enum TaskResult { RESULT_NONE, RESULT_SUCCESS, RESULT_TIMEOUT };
TaskResult last_result = RESULT_NONE;
unsigned long result_start_time = 0; 


#define WRIST1_PWM 5
#define WRIST1_DIR 28
#define WRIST1_ENCA 37
#define WRIST1_ENCB 35

#define WRIST2_PWM 4
#define WRIST2_DIR 26
#define WRIST2_ENCA 41
#define WRIST2_ENCB 40

#define GRIPPY_PWM 2
#define GRIPPY_DIR 22
#define GRIPPY_ENCA 44
#define GRIPPY_ENCB 45

#define LA1_PWM 11
#define LA1_DIR 49
#define LA1_POTEN A7

#define LA2_PWM 10
#define LA2_DIR 48
#define LA2_POTEN A6

// < -------------- MICRO ROS DEFINE --------------->
rcl_publisher_t publisher2;
rcl_subscription_t subscriber2;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator = rcl_get_default_allocator();
rcl_node_t node;
rcl_timer_t timer;

rcl_subscription_t subscriber;
std_msgs__msg__String msg_sub;
rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray msg_pub;

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
    arm_encoder_msg.data.data[0] = LA1_enc;
    arm_encoder_msg.data.data[1] = LA2_enc;
    arm_encoder_msg.data.data[2] = arm_posi[0];
    arm_encoder_msg.data.data[3] = arm_posi[1];
    arm_encoder_msg.data.data[4] = arm_posi[2];

    RCSOFTCHECK(rcl_publish(&publisher2, &arm_encoder_msg, NULL));
  }
}

void subscription_callback_arm(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *msg =
    (const std_msgs__msg__Int32MultiArray *)msgin;

  // Copy received ARM PWM values
  for (int i = 0; i < 5; i++) {
    arm_pwm_values[i] = msg->data.data[i];
  }

  last_message_time_arm = millis();
}

void subscription_callback(const void * msgin) {
  const std_msgs__msg__String * received_msg = (const std_msgs__msg__String *)msgin;
  
  if (received_msg->data.size > 0) {
    char cmd = received_msg->data.data[0];

    
    command_active = true;
    command_start_time = millis();
    last_result = RESULT_NONE; 

    if (cmd == 'h') { // HOME (1, 1)
      digitalWrite(PIN_SEND_VP, HIGH);
      digitalWrite(PIN_SEND_VN, HIGH);
    }
    else if (cmd == 'c') { // CRAB (0, 1)
      digitalWrite(PIN_SEND_VP, LOW);
      digitalWrite(PIN_SEND_VN, HIGH);
    }
    else if (cmd == 'n') { // 90 DEGREE (1, 0)
      digitalWrite(PIN_SEND_VP, HIGH);
      digitalWrite(PIN_SEND_VN, LOW);
    }
   
  }
}

// < ----------- SETUP -------------------->

void setup() {


  pinMode(PIN_SEND_VP, OUTPUT);
  pinMode(PIN_SEND_VN, OUTPUT);
  pinMode(PIN_FEEDBACK, INPUT); 
  
  digitalWrite(PIN_SEND_VP, LOW);
  digitalWrite(PIN_SEND_VN, LOW);
  
  set_microros_transports();
  Serial.begin(115200);
  // Setup arm encoders
 

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

  msg_sub.data.capacity = 10; 
  msg_sub.data.data = (char*) malloc(msg_sub.data.capacity * sizeof(char));
  msg_sub.data.size = 0;

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/cmd_pivot"
  );

  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/cmd_pivot_feedback"
  );

  msg_pub.data.capacity = 2;
  msg_pub.data.size = 2;
  msg_pub.data.data = (float*) malloc(2 * sizeof(float));


  // Timer setup
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Executor setup
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber2, &arm_pwm_msg, &subscription_callback_arm, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  uint32_t current_time = millis();

  //<----------------- ARM TEST --------------->

  if (current_time - last_message_time_arm > 1000) {
    //only running loop till 4 beacuse gripper has no feedback as of now
    for (int i = 0; i < 5; i++) {
      arm_pwm_values[i] = 0;
    }
  }

  if (command_active) {
    unsigned long time_elapsed = millis() - command_start_time;

    //
    if (time_elapsed > SAFETY_TIMEOUT) {
       
       digitalWrite(PIN_SEND_VP, HIGH);
       digitalWrite(PIN_SEND_VN, HIGH);
       command_active = false; 
       
       
       last_result = RESULT_TIMEOUT;
       result_start_time = millis();
    }
    // CASE B: NORMAL CHECK
    else if (time_elapsed > FEEDBACK_IGNORE_TIME) {
      if (digitalRead(PIN_FEEDBACK) == LOW) {
        // Success!
        digitalWrite(PIN_SEND_VP, LOW);
        digitalWrite(PIN_SEND_VN, LOW);
        command_active = false; 
        
      
        last_result = RESULT_SUCCESS;
        result_start_time = millis();
      }
    }
  }

  if (command_active) {
    // STATE: MOVING [1.0, 0.0]
    msg_pub.data.data[0] = 1.0;
    msg_pub.data.data[1] = 0.0;
  }
  else {
    // Not active, check if we are in the "5-second hold" period
    if (last_result != RESULT_NONE && (millis() - result_start_time < POST_TASK_DELAY)) {
       if (last_result == RESULT_SUCCESS) {
         // STATE: DONE [0.0, 1.0] (Held for 5s)
         msg_pub.data.data[0] = 0.0;
         msg_pub.data.data[1] = 1.0;
       } 
       else if (last_result == RESULT_TIMEOUT) {
         // STATE: FAILED [1.0, 1.0] (Held for 5s)
         msg_pub.data.data[0] = 1.0;
         msg_pub.data.data[1] = 1.0;
       }
    } 
    else {
       // STATE: IDLE [0.0, 0.0] (Default)
       // This runs after the 5s hold is over, or if stopped manually
       msg_pub.data.data[0] = 0.0;
       msg_pub.data.data[1] = 0.0;
    }
  }

  LA1_enc = analogRead(LA1_POTEN);
  LA2_enc = analogRead(LA2_POTEN);

  LA1.run(arm_pwm_values[0]);
  

  LA2.run(arm_pwm_values[1]);

  wrist1_motor.run(arm_pwm_values[2]);
  wrist2_motor.run(arm_pwm_values[3]);
  grippy_motor.run(arm_pwm_values[4]);

  rcl_publish(&publisher, &msg_pub, NULL);

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  
}


// void readEncoderArm1() {
//   static int lastENCA = LOW;
//   int currentENCA = digitalRead(WRIST1_ENCA);
//   int currentENCB = digitalRead(WRIST1_ENCB);

//   // Determine direction based on ENCA and ENCB
//   if (currentENCA != lastENCA) {
//     if (currentENCA == HIGH) {
//       if (currentENCB == LOW) {
//         arm_posi[0]++;
//       } else {
//         arm_posi[0]--;
//       }
//     } else {
//       if (currentENCB == LOW) {
//         arm_posi[0]--;
//       } else {
//         arm_posi[0]++;
//       }
//     }
//     lastENCA = currentENCA;
//   }
// }

// void readEncoderArm2() {
//   static int lastENCA = LOW;
//   int currentENCA = digitalRead(WRIST2_ENCA);
//   int currentENCB = digitalRead(WRIST2_ENCB);

//   // Determine direction based on ENCA and ENCB
//   if (currentENCA != lastENCA) {
//     if (currentENCA == HIGH) {
//       if (currentENCB == LOW) {
//         arm_posi[1]++;
//       } else {
//         arm_posi[1]--;
//       }
//     } else {
//       if (currentENCB == LOW) {
//         arm_posi[1]--;
//       } else {
//         arm_posi[1]++;
//       }
//     }
//     lastENCA = currentENCA;
//   }
// }

// void readEncoderArm3() {
//   static int lastENCA = LOW;
//   int currentENCA = digitalRead(GRIPPY_ENCA);
//   int currentENCB = digitalRead(GRIPPY_ENCB);

//   // Determine direction based on ENCA and ENCB
//   if (currentENCA != lastENCA) {
//     if (currentENCA == HIGH) {
//       if (currentENCB == LOW) {
//         arm_posi[2]++;
//       } else {
//         arm_posi[2]--;
//       }
//     } else {
//       if (currentENCB == LOW) {
//         arm_posi[2]--;
//       } else {
//         arm_posi[2]++;
//       }
//     }
//     lastENCA = currentENCA;
//   }
// }