#include <Arduino.h>
#include "driver/pcnt.h"

// --- PIN DEFINITIONS ---
#define input_2 39
#define input_1 36
#define feedback_pin 2

const int pwm[4] = { 5, 17, 16, 22 };
//const int pwm[4] = {4,17,25,27};
const int dir[4] = { 15, 21, 4, 23 };
const int ENC_A[4] = { 25, 27, 19, 33 };
const int ENC_B[4] = { 26, 14, 18, 32 };
int target_angle[4] = { 0, 0, 0, 0 };
int mode = 0;
unsigned long time_in_cycle[4] = { 0, 0, 0, 0 };


int16_t count[4] = { 0, 0, 0, 0 };
long target_counts[4] = { 0, 0, 0, 0 };
long error[4] = { 0, 0, 0, 0 };
static unsigned long lastPrint[4] = { 0, 0, 0, 0 };
// --- SETTINGS ---
// Note: 102,000 counts per rev seems very high.
// If the wheel spins too much or too little, adjust this number.
const float COUNTS_PER_REV = 95520;

const int MOTOR_SPEED = 200;
const int TOLERANCE = 20;

// --- VARIABLES ---
volatile long current_position[4] = { 0, 0, 0, 0 };  // Start at 0

pcnt_unit_t pcnt_unit_1 = PCNT_UNIT_0;
pcnt_unit_t pcnt_unit_2 = PCNT_UNIT_1;
pcnt_unit_t pcnt_unit_3 = PCNT_UNIT_2;
pcnt_unit_t pcnt_unit_4 = PCNT_UNIT_3;

pcnt_unit_t pcnt_array[4] = { pcnt_unit_1, pcnt_unit_2, pcnt_unit_3, pcnt_unit_4 };

void setupEncoder(int ENC1_A, int ENC1_B, pcnt_unit_t pcnt_unit) {
  pcnt_config_t cfg = {};
  cfg.pulse_gpio_num = ENC1_A;
  cfg.ctrl_gpio_num = ENC1_B;
  cfg.lctrl_mode = PCNT_MODE_REVERSE;
  cfg.hctrl_mode = PCNT_MODE_KEEP;
  cfg.pos_mode = PCNT_COUNT_INC;
  cfg.neg_mode = PCNT_COUNT_DEC;
  cfg.counter_h_lim = 30000;
  cfg.counter_l_lim = -30000;
  cfg.unit = pcnt_unit;
  cfg.channel = PCNT_CHANNEL_0;

  pcnt_unit_config(&cfg);
  pcnt_counter_pause(pcnt_unit);
  pcnt_counter_clear(pcnt_unit);
  pcnt_counter_resume(pcnt_unit);
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
    pinMode(input_1, INPUT);
    pinMode(input_2, INPUT);
    pinMode(feedback_pin, OUTPUT);
    setupEncoder(ENC_A[i], ENC_B[i], pcnt_array[i]);
  }
}

void loop() {
  if (digitalRead(input_1) == HIGH && digitalRead(input_2) == HIGH) {
    mode = 1;
  } else if (digitalRead(input_1) == LOW && digitalRead(input_2) == HIGH) {
    mode = 2;
  } else if (digitalRead(input_1) == HIGH && digitalRead(input_2) == LOW) {
    mode = 3;
  }
  switch (mode) {
    case 1:
      target_angle[0] = 0;
      target_angle[1] = 0;
      target_angle[2] = 0;
      target_angle[3] = 0;
      break;
    case 2:
      target_angle[0] = 60;
      target_angle[1] = -60;
      target_angle[2] = 60;
      target_angle[3] = -60;
      break;
    case 3:
      target_angle[0] = 90;
      target_angle[1] = -90;
      target_angle[2] = 90;
      target_angle[3] = -90;
      break;
  }
  bool flag = false;

  for (int i = 0; i < 4; i++) {
    time_in_cycle[i] = millis();
    // --- 2. ENCODER READING ---
    // add loop
    pcnt_get_counter_value(pcnt_array[i], &count[i]);
    pcnt_counter_clear(pcnt_array[i]);
    current_position[i] += count[i];

    // --- 3. CALCULATE TARGET IN COUNTS ---
    target_counts[i] = (long)((target_angle[i] / 360.0) * 41790);
   
    // --- 4. CONTROL LOGIC ---
    error[i] = target_counts[i] - current_position[i];

    if (abs(error[i]) <= TOLERANCE) {
      // Close enough -> STOP
      flag = true;
      digitalWrite(feedback_pin, LOW);
      setMotor(0, true, pwm[i], dir[i]);
    } else if (error[i] > 0) {
      flag = false;
      digitalWrite(feedback_pin, HIGH);
      // Target is ahead -> Forward
      setMotor(MOTOR_SPEED, true, pwm[i], dir[i]);
    } else {
      flag = false;
      digitalWrite(feedback_pin, HIGH);
      // Target is behind -> Backward
      setMotor(MOTOR_SPEED, false, pwm[i], dir[i]);
    }

    // --- 5. DEBUGGING ---
    if (millis() - lastPrint[i] > 1000) {
      Serial.printf("Motor No: %d", i);
      Serial.print("Target Angle: ");
      Serial.print(target_angle[i]);
      Serial.print(" | Target Cnt: ");
      Serial.print(target_counts[i]);
      Serial.print(" | Current Cnt: ");
      Serial.println(current_position[i]);
      lastPrint[i] = millis();
    }
  }
  if(flag) mode = 0;
  delay(10);
}