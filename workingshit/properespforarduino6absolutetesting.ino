#include <Arduino.h>
#include "driver/pcnt.h"
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#define MOTOR_SPEED 200
#define input_2 39 //VN
#define input_1 36 //VP
#define feedback_pin 2
//115 wheel 4
int TOLERANCE = 50;

const int limitSwitchPin[4] = { 13, 12, 35, 34 };
bool stopped[4] = { false, false, false, false };
const int pwm[4] = { 5, 17, 16, 22 };
const int dir[4] = { 15, 21, 4, 23 };
const int ABS_ENC_PIN[4] = { 26, 14, 25, 33 };
int target_angle[4] = { 1185, 1290,1695, 1787 };
unsigned long time_in_cycle[4] = { 0, 0, 0, 0 };
int16_t count[4] = { 0, 0, 0, 0 };
long target_counts[4] = { 0, 0, 0, 0 };
long error[4] = { 0, 0, 0, 0 };


volatile long current_position[4] = { 0, 0, 0, 0 };  // Start at 0


rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray msg;
float msg_data[4]; // Buffer for incoming array
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node; 

void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  
  // Ensure we received exactly 4 angles
  if (msg->data.size >= 4) {
    float angles[4];
    for(int i = 0; i < 4; i++) {
      angles[i] = msg->data.data[i];
    }

    // Apply hardware home offsets
    angles[0] += 104.0; // Wheel 1 offset
    angles[1] += 113.0;
    angles[2] += 150.0; // Wheel 1 offset
    angles[3] += 157.0;
     // Wheel 3 offset

    for(int i = 0; i < 4; i++) {
      // Wrap angles between 0 and 360
      while(angles[i] >= 360.0) angles[i] -= 360.0;
      while(angles[i] < 0.0) angles[i] += 360.0;

      // Map 0-360 degrees to 0-4095 absolute encoder counts
      target_angle[i] = (int)((angles[i] / 360.0) * 4095.0);
    }
    // Lock into ROS mode so loop() doesn't overwrite these targets
   
  }
}

void setMotor(int speed, bool direction, int M_PWM, int M_DIR) {
  //make pin compatible
  if (speed == 0) {
    analogWrite(M_PWM, 0);
    return;
  }
  digitalWrite(M_DIR, direction ? LOW : HIGH);
  analogWrite(M_PWM, speed);
}





void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    pinMode(pwm[i], OUTPUT);
    pinMode(dir[i], OUTPUT);
    
  }
  pinMode(ABS_ENC_PIN[0],INPUT);   
  pinMode(ABS_ENC_PIN[1],INPUT);   
  pinMode(ABS_ENC_PIN[2],INPUT);   
  pinMode(ABS_ENC_PIN[3],INPUT);   

  // pinMode(input_1, INPUT);
  // pinMode(input_2, INPUT);
  // pinMode(feedback_pin, OUTPUT);

  set_microros_transports(); // Initializes transport (default over Serial)
  
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "rover_pivot_node", "", &support);

  // Configure message memory
  msg.data.capacity = 4;
  msg.data.data = msg_data;
  msg.data.size = 0;

  // Create subscriber for a Float32MultiArray on topic "target_angles"
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "target_angles"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
}




void loop() { 
 
  // if (digitalRead(input_1) == HIGH && digitalRead(input_2) == HIGH) {
  //   mode =0;
  // } else if (digitalRead(input_1) == LOW && digitalRead(input_2) == HIGH) {
  //   mode = 1; 
  // } else if (digitalRead(input_1) == HIGH && digitalRead(input_2) == LOW) {
  //   mode = 2; 
  // } else {
  //   mode = 0; 
  // }
  
  // Serial.println("Target"+ String(target_angle[0]) + " " + String(target_angle[1]) + " " + String(target_angle[2]) + " " + String(target_angle[3]));
  // Serial.println("Current"+String(analogRead(26)) + " " + String(analogRead(14)) + " " + String(analogRead(25)) + " " + String(analogRead(33)));
  
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  for (int i = 0; i < 4; i++) {
    
      current_position[i] = analogRead(ABS_ENC_PIN[i]);
      time_in_cycle[i] = millis();
      
    
      

      // --- 3. CALCULATE TARGET IN COUNTS ---
      target_counts[i] = target_angle[i] ; 
      // --- 4. CONTROL LOGIC ---
      error[i] = target_counts[i] - current_position[i];

      if (abs(error[i]) <= TOLERANCE) {
        // Close enough -> STOP
      
        setMotor(0, true, pwm[i], dir[i]);
      } else if ((error[i] > 0)) {
        digitalWrite(feedback_pin, HIGH);
        // Target is ahead -> Forward
        setMotor(MOTOR_SPEED, false, pwm[i], dir[i]);
      } else {
        digitalWrite(feedback_pin, HIGH);
        // Target is behind -> Backward
        setMotor(MOTOR_SPEED, true, pwm[i], dir[i]);
      }
    
  
  // if(error[0]<=TOLERANCE&&error[1]<=TOLERANCE&&error[2]<=TOLERANCE&&error[3]<=TOLERANCE){
  //    digitalWrite(feedback_pin, LOW);
     
  // }
 
  
}
 if(abs(error[1])<=TOLERANCE&&abs(error[3])<=TOLERANCE){
     digitalWrite(feedback_pin, LOW);     
  }
}


//while implementing subscriber for pivot ackerman , just add tolerance change in the subscriber .