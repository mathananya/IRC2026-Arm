#include <Arduino.h>
#include "driver/pcnt.h"

#define MOTOR_SPEED 200
#define input_2 39 //VN
#define input_1 36 //VP
#define feedback_pin 2

const int TOLERANCE = 20;
int mode = 0;
const int limitSwitchPin[4] = { 13, 12, 35, 34 };
bool stopped[4] = { false, false, false, false };
const int pwm[4] = { 5, 17, 16, 22 };
const int dir[4] = { 15, 21, 4, 23 };
const int ENC_A[4] = { 26, 14, 19, 33 };
const int ENC_B[4] = { 25, 27, 18, 32 };
int target_angle[4] = { 0, 0, 0, 0 };
unsigned long time_in_cycle[4] = { 0, 0, 0, 0 };
int16_t count[4] = { 0, 0, 0, 0 };
long target_counts[4] = { 0, 0, 0, 0 };
long error[4] = { 0, 0, 0, 0 };
static unsigned long lastPrint[4] = { 0, 0, 0, 0 };
const float COUNTS_PER_REV = 95520;
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


void gohome() {
  while (true) {
    for (int i = 0; i < 4; i++) {
      if (digitalRead(limitSwitchPin[i]) == HIGH) { 
      
        if (digitalRead(limitSwitchPin[i]) == HIGH) {  // debounce
          //Serial.printf("%d Motor reached home\n", i);
          pcnt_counter_clear(pcnt_array[i]);
          setMotor(0, true, pwm[i], dir[i]);
          stopped[i] = true;
        }
      }
      else if(stopped[i] == false) {
        digitalWrite(feedback_pin, HIGH);
        if (i == 3 || i==0)   {
          setMotor(MOTOR_SPEED, true, pwm[i], dir[i]);
        } else {
          setMotor(MOTOR_SPEED, false, pwm[i], dir[i]);
        }
      }
    }
    if(stopped[0] && stopped[1] && stopped[2] && stopped[3]){
        current_position[0] = 0;
        current_position[1] = 0;
        current_position[2] = 0;
        current_position[3] = 0;
        target_counts[0] = 0;
        target_counts[1] = 0;
        target_counts[2] = 0;
        target_counts[3] = 0;
        target_angle[0] = 0;
        target_angle[1] = 0;
        target_angle[2] = 0;
        target_angle[3] = 0;
        digitalWrite(feedback_pin, LOW);
        break;
    }
  }

}


void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    pinMode(pwm[i], OUTPUT);
    pinMode(dir[i], OUTPUT);
    pinMode(limitSwitchPin[i], INPUT_PULLUP);
    pinMode(input_1, INPUT);
    pinMode(input_2, INPUT);
    setupEncoder(ENC_A[i], ENC_B[i], pcnt_array[i]);
  }
  pinMode(feedback_pin, OUTPUT);
  gohome();
  // put your setup code here, to run once:
}




void loop() { 
  /*Serial.print(digitalRead(input_1));
  Serial.print(digitalRead(input_2));
  Serial.println("");*/
  if (digitalRead(input_1) == HIGH && digitalRead(input_2) == HIGH) {
    stopped[0] = false;
    stopped[1] = false;
    stopped[2] = false;
    stopped[3] = false;
    gohome();
  } else if (digitalRead(input_1) == LOW && digitalRead(input_2) == HIGH) {
    mode = 1; 
  } else if (digitalRead(input_1) == HIGH && digitalRead(input_2) == LOW) {
    mode = 2; 
  } else {
    mode = 0; 
  }
    switch (mode) {
    case 1:
      target_angle[0] = -60;
      target_angle[1] = 60;
      target_angle[2] = 60;
      target_angle[3] = -60;
      break;
    case 2:
      target_angle[0] = -90;
      target_angle[1] = 90;
      target_angle[2] = 90;
      target_angle[3] = -90;
      break;
  }
  for (int i = 0; i < 4; i++) {
    time_in_cycle[i] = millis();
    pcnt_get_counter_value(pcnt_array[i], &count[i]);
    pcnt_counter_clear(pcnt_array[i]);
    current_position[i] += count[i];

    // --- 3. CALCULATE TARGET IN COUNTS ---
    target_counts[i] = (long)((target_angle[i] / 360.0) * 41790); 
    // --- 4. CONTROL LOGIC ---
    error[i] = target_counts[i] - current_position[i];

    if (abs(error[i]) <= TOLERANCE) {
      // Close enough -> STOP
     
      setMotor(0, true, pwm[i], dir[i]);
    } else if (error[i] > 0) {
      digitalWrite(feedback_pin, HIGH);
      // Target is ahead -> Forward
      setMotor(MOTOR_SPEED, true, pwm[i], dir[i]);
    } else {
      digitalWrite(feedback_pin, HIGH);
      // Target is behind -> Backward
      setMotor(MOTOR_SPEED, false, pwm[i], dir[i]);
    }
    
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
  if(error[0]<=TOLERANCE&&error[1]<=TOLERANCE&&error[2]<=TOLERANCE&&error[3]<=TOLERANCE){
     digitalWrite(feedback_pin, LOW);
  }
  
}


