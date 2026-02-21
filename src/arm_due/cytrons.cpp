#include "cytrons.h"
#include <math.h>

MDD10A* MDD10A::_instance = nullptr;

MDD10A::MDD10A(int pwm, int dir, int enca, int encb)
  : _pwm_pin(pwm), _dir_pin(dir), _enca(enca), _encb(encb), _encoderCount(0) {
  pinMode(_pwm_pin, OUTPUT);
  pinMode(_dir_pin, OUTPUT);
  pinMode(_enca, INPUT_PULLUP);
  pinMode(_encb, INPUT_PULLUP);

  if (enca == 0 && encb == 0) {
    _instance = this;
  }
}

MDD10A::~MDD10A() {
  stop();
}

void MDD10A::run(int pwr) {
  int dir = (pwr > 0) ? 1 : 0; // Setting to HIGH when pwr is +ve, setting to LOW when pwr is 0 / -ve
  int u = min(abs(pwr), 255);
  
  analogWrite(_pwm_pin, u);
  digitalWrite(_dir_pin, dir);
}

void MDD10A::stop() {
  analogWrite(_pwm_pin, 0); // Setting PWM to 0 to stop the motor
  digitalWrite(_dir_pin, LOW); // Setting DIR to LOW to not waste any energy after stopping
}

void MDD10A::encoderInterruptA() {
  if (_instance) {
    if (digitalRead(_instance->_enca) == digitalRead(_instance->_encb)) {
      _instance->_encoderCount++;
    } else {
      _instance->_encoderCount--;
    }
  }
}

void MDD10A::attachEncInterrupt() {
  attachInterrupt(digitalPinToInterrupt(_enca), encoderInterruptA, CHANGE);
}

int MDD10A::getEncoderCount() const {
  return _encoderCount;
}

// <-------------- MDD3A Definition -------------------->

MDD3A::MDD3A(int pin1, int pin2, int pin3, int pin4)
  : _pin1(pin1), _pin2(pin2), _pin3(pin3), _pin4(pin4), 
    _currentSteps(0), _isRunning(false), _lastStepTime(0), 
    _targetSteps(0), _stepDelay(1000) {
  
  pinMode(_pin1, OUTPUT);
  pinMode(_pin2, OUTPUT);
  pinMode(_pin3, OUTPUT);
  pinMode(_pin4, OUTPUT);
  
  stop();
}

MDD3A::~MDD3A() {
  stop();
}

void MDD3A::run(int dir, int moveTo) {
  int targetSteps = map(moveTo, -100, 100, -1667, 1667); // Internal limit angle of -100 to 100 degrees (approx)

  _dir = targetSteps > _currentSteps ? 1 : -1;
  _isRunning = true; // Set running state
  _targetSteps = targetSteps;
}

void MDD3A::update() {
  if (!_isRunning) return; // If not running, exit

  unsigned long currentTime = micros(); // Get current time
  int stepsToTake = (_dir == 1) ? _targetSteps - _currentSteps : _currentSteps - _targetSteps;

  if (currentTime - _lastStepTime >= _stepDelay) {     
    if (stepsToTake > 0) {
      if (_dir == 1) {
        setPins(_steps[(stepsToTake) % 4]); // Move in the set direction
      } else if (_dir == -1) {
        setPins(_reversedSteps[(stepsToTake) % 4]); // Move in the set direction
      }
      stepsToTake--; // Decrease steps left
      _lastStepTime = currentTime; // Update last step time
      _currentSteps += _dir;

    } else {
      stop(); // Stop if no steps left
      _targetSteps = 0;
    }

  }
}

void MDD3A::stop() {
  digitalWrite(_pin1, LOW);
  digitalWrite(_pin2, LOW);
  digitalWrite(_pin3, LOW);
  digitalWrite(_pin4, LOW);
  
  _isRunning = false; // Reset running state
}

void MDD3A::setPins(const int states[4]) {
  digitalWrite(_pin1, states[0]);
  digitalWrite(_pin2, states[1]);
  digitalWrite(_pin3, states[2]);
  digitalWrite(_pin4, states[3]);
}

int MDD3A::getTargetSteps() { return _targetSteps; }
bool MDD3A::getIsRunning() { return _isRunning; }