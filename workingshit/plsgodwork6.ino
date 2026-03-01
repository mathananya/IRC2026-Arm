#include <Arduino.h>
#include "driver/pcnt.h"

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h> 
#include <std_msgs/msg/string.h> 

// --- PIN DEFINITIONS ---
#define M1_PWM 13//4
#define M1_DIR 21//15
#define ENC1_A 33 
#define ENC1_B 32

#define M2_PWM 16//17
#define M2_DIR 4//16
#define ENC2_A 34
#define ENC2_B 35

#define M3_PWM 2//15
#define M3_DIR 15//26
#define ENC3_A 5
#define ENC3_B 18

#define M4_PWM  0  //27
#define M4_DIR 12//14
#define ENC4_A 23
#define ENC4_B 19

const float WHEEL_RADIUS = 0.2;
const float ENC_PPR = 1410.0;
const int32_t ENCODER_LIMIT = 1410*10000; 

// PID Constants
float Kp = 75.0;
float Ki = 0.00;
float Kd = 0.03;

// Raw PWM Constant
// Multiplier to convert target_v (m/s) to PWM. 
// Assuming target 1.0 = 255 PWM. Adjust if needed.
const float PWM_GAIN = 200.0; 

float target_v1 = 0.0;
float target_v2 = 0.0;
float target_v3 = 0.0;
float target_v4 = 0.0;

float filtered_v1=0.0;
float filtered_v2=0.0;
float filtered_v3=0.0;
float filtered_v4=0.0;
float alpha=0.6;
// --- LOGIC CONTROL ---
bool use_pid = true; // true = Home (PID), false = Crab/90 (Raw PWM)

// --- ACCUMULATED TICKS (ODOMETRY) ---
long long enc1_total = 0;
long long enc2_total = 0;
long long enc3_total = 0;
long long enc4_total = 0;

class MDD10A {
public:
  MDD10A(int pwm, int dir) : _pwm(pwm), _dir(dir) {
    pinMode(_pwm, OUTPUT);
    pinMode(_dir, OUTPUT);
  }
  void run(int pwr) {
    pwr = constrain(pwr, -255, 255);
    // Deadband check
    if (abs(pwr) < 15) pwr = 0;
    digitalWrite(_dir, (pwr >= 0) ? HIGH : LOW);
    analogWrite(_pwm, abs(pwr));
  }
private:
  int _pwm, _dir;
};

MDD10A motor1(M1_PWM, M1_DIR);
MDD10A motor2(M2_PWM, M2_DIR);
MDD10A motor3(M3_PWM, M3_DIR);
MDD10A motor4(M4_PWM, M4_DIR); 

pcnt_unit_t pcnt1 = PCNT_UNIT_0;
pcnt_unit_t pcnt2 = PCNT_UNIT_1;
pcnt_unit_t pcnt3 = PCNT_UNIT_2;
pcnt_unit_t pcnt4 = PCNT_UNIT_3; 

double out1=0, out2=0, out3=0, out4=0; 
double ie1=0, ie2=0, ie3=0, ie4=0;     
double pe1=0, pe2=0, pe3=0, pe4=0;   
unsigned long prevT_us = 0;

// --- ROS OBJECTS ---
rcl_node_t node;
rcl_subscription_t sub;       
rcl_subscription_t sub_pivot; 
rcl_publisher_t pub_enc; 
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

std_msgs__msg__Float32MultiArray msg_sub; 
float sub_data[4]; 

std_msgs__msg__Int32MultiArray msg_enc; 
int32_t enc_data[4];

std_msgs__msg__String msg_pivot;

unsigned long last_cmd_time = 0;

void setupEncoder(pcnt_unit_t unit, int A, int B) {
  pcnt_config_t cfg = {};
  cfg.pulse_gpio_num = A;
  cfg.ctrl_gpio_num  = B;
  cfg.lctrl_mode     = PCNT_MODE_REVERSE;
  cfg.hctrl_mode     = PCNT_MODE_KEEP;
  cfg.pos_mode       = PCNT_COUNT_INC;
  cfg.neg_mode       = PCNT_COUNT_DEC;
  cfg.counter_h_lim  = 30000;
  cfg.counter_l_lim  = -30000;
  cfg.unit           = unit;
  cfg.channel        = PCNT_CHANNEL_0;

  pcnt_unit_config(&cfg);
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}



void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * received_msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  
  if (received_msg->data.size >= 4) {
    target_v1 = received_msg->data.data[0];
    target_v2 = received_msg->data.data[1];
    target_v3 = received_msg->data.data[2];
    target_v4 = received_msg->data.data[3];
    last_cmd_time = millis();
  }
}

void pivot_callback(const void * msgin) {
  const std_msgs__msg__String * received_msg = (const std_msgs__msg__String *)msgin;

  if (received_msg->data.size > 0) {
    char cmd = received_msg->data.data[0];
    
    if (cmd == 'h') {
      use_pid = true; // HOME -> Use PID
    } 
    else if (cmd == 'c' || cmd == 'n') {
      use_pid = false; // CRAB/90 -> Use Raw PWM
      
      // Reset Integral Error so it doesn't "explode" while in Raw mode
      ie1=0; ie2=0; ie3=0; ie4=0;
      out1=0; out2=0; out3=0; out4=0;
    }
  }
}

void setup() {
  Serial.begin(115200);

  setupEncoder(pcnt1, ENC1_A, ENC1_B);
  setupEncoder(pcnt2, ENC2_A, ENC2_B);
  setupEncoder(pcnt3, ENC3_A, ENC3_B);
  setupEncoder(pcnt4, ENC4_A, ENC4_B); 

  set_microros_transports();
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "four_wheel_drive", "", &support); 

  msg_sub.data.capacity = 4;
  msg_sub.data.data = sub_data;
  msg_sub.data.size = 0;

  rclc_subscription_init_default(
    &sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/wheel_speeds"
  );

  msg_pivot.data.capacity = 10;
  msg_pivot.data.data = (char*) malloc(msg_pivot.data.capacity * sizeof(char));
  msg_pivot.data.size = 0;

  rclc_subscription_init_default(
    &sub_pivot,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/cmd_pivot"
  );

  msg_enc.data.capacity = 4;
  msg_enc.data.data = enc_data;
  msg_enc.data.size = 4;

  rclc_publisher_init_default(
    &pub_enc,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/wheel_encoders"
  );

  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(&executor, &sub, &msg_sub, &subscription_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_pivot, &msg_pivot, &pivot_callback, ON_NEW_DATA);

  prevT_us = micros();
}

void loop() {

  
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  
  if (millis() - last_cmd_time > 200) {
    target_v1 = target_v2 = target_v3 = target_v4 = 0.0;
    out1 = out2 = out3 = out4 = 0; 
    ie1 = ie2 = ie3 = ie4 = 0; 
    pe1 = pe2 = pe3 = pe4 = 0;
   motor1.run(0);
     motor2.run(0);
    motor3.run(0);
     motor4.run(0);

  }

  unsigned long currT = micros();
  float dt = (currT - prevT_us) / 1e6;
  if (dt <= 0 || dt > 0.1) {
    prevT_us = currT;
    return;
  }

  // --- 1. READ ENCODERS (Running Total) ---
  int16_t c1 = 0; pcnt_get_counter_value(pcnt1, &c1); pcnt_counter_clear(pcnt1); 
  enc1_total = (enc1_total + c1) % ENCODER_LIMIT; 
  
  int16_t c2 = 0; pcnt_get_counter_value(pcnt2, &c2); pcnt_counter_clear(pcnt2);
  enc2_total = (enc2_total + c2) % ENCODER_LIMIT;

  int16_t c3 = 0; pcnt_get_counter_value(pcnt3, &c3); pcnt_counter_clear(pcnt3);
  enc3_total = (enc3_total + c3) % ENCODER_LIMIT;

  int16_t c4 = 0; pcnt_get_counter_value(pcnt4, &c4); pcnt_counter_clear(pcnt4);
  enc4_total = (enc4_total + c4) % ENCODER_LIMIT;

  // --- 2. CONTROL LOGIC ---
  
  if (use_pid) {
    // === HOME MODE: PID CONTROL ===
    
    // Motor 1
    double v1 = ((double)c1 / dt / ENC_PPR) * (2.0 * PI * WHEEL_RADIUS);
     filtered_v1= (alpha * v1) + ((1.0 - alpha) * filtered_v1);
    double e1 = target_v1 - filtered_v1;
    ie1 += e1 * dt; ie1 = constrain(ie1, -20, 20);
    out1 += (Kp * e1) + (Ki * ie1) + (Kd * (e1 - pe1) / dt);
   motor1.run((int)(target_v1 * PWM_GAIN));; pe1 = e1;

    // Motor 2
    double v2 = ((double)c2 / dt / ENC_PPR) * (2.0 * PI * WHEEL_RADIUS);
    filtered_v2= (alpha * v2) + ((1.0 - alpha) * filtered_v2);
    double e2 = target_v2 - filtered_v2;
    ie2 += e2 * dt; ie2 = constrain(ie2, -20, 20);
    out2 += (Kp * e2) + (Ki * ie2) + (Kd * (e2 - pe2) / dt);
    motor2.run(-(int)(target_v2 * PWM_GAIN));; pe2 = e2;

    // Motor 3
    double v3 = ((double)c3 / dt / ENC_PPR) * (2.0 * PI * WHEEL_RADIUS);
    filtered_v3= (alpha * v3) + ((1.0 - alpha) * filtered_v3);
    double e3 = target_v3 - filtered_v3;
    ie3 += e3 * dt; ie3 = constrain(ie3, -20, 20);
    out3 += (Kp * e3) + (Ki * ie3) + (Kd * (e3 - pe3) / dt);
  motor3.run((int)(target_v3 * PWM_GAIN));; pe3 = e3;

    // Motor 4
    double v4 = ((double)c4 / dt / ENC_PPR) * (2.0 * PI * WHEEL_RADIUS);
    filtered_v4= (alpha * v4) + ((1.0 - alpha) * filtered_v4);
    double e4 = target_v4 - filtered_v4;
    ie4 += e4 * dt; ie4 = constrain(ie4, -20, 20);
    out4 += (Kp * e4) + (Ki * ie4) + (Kd * (e4 - pe4) / dt);
   motor4.run(-(int)(target_v4 * PWM_GAIN)); pe4 = e4;
    
  } else {
    // === CRAB / 90 MODE: RAW PWM MAPPING ===
    // target_v (m/s) -> PWM (0-255)
    
    motor1.run((int)(target_v1 * PWM_GAIN));
    motor2.run(-(int)(target_v2 * PWM_GAIN));
    motor3.run((int)(target_v3 * PWM_GAIN));
    motor4.run(-(int)(target_v4 * PWM_GAIN));
    
    // We do NOT update 'pe' (prev error) or 'ie' (integral error) here
  }

  // --- 3. PUBLISH ODOMETRY ---
  msg_enc.data.data[0] = (int32_t)enc1_total;
  msg_enc.data.data[1] = (int32_t)enc2_total;
  msg_enc.data.data[2] = (int32_t)enc3_total;
  msg_enc.data.data[3] = (int32_t)enc4_total;
  
  rcl_publish(&pub_enc, &msg_enc, NULL);

  prevT_us = currT;
  delay(10);
}