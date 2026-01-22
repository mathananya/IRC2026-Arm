#ifndef CYTRONS_H
#define CYTRONS_H

#include "Arduino.h"

class MDD10A {
  public:
    MDD10A(int pwm, int dir, int enca, int encb);
    ~MDD10A();

    void run(int pwr);
    void stop();
    void attachEncInterrupt();
    int getEncoderCount() const;

  private:
    int _pwm_pin;
    int _dir_pin;
    int _enca;
    int _encb;

    volatile int _encoderCount; // Shared variable for ISR
    static void encoderInterruptA(); // Static ISR for A

    static MDD10A* _instance; // Pointer to current instance for ISRs
};

class MDD3A {
  public:
    MDD3A(int pin1, int pin2, int pin3, int pin4);
    ~MDD3A();

    void run(int dir, int steps); // 1 -> CLOCKWISE, -1 -> ANTICLOCKWISE
    void stop();
    void update(); // New method for non-blocking updates

    int getTargetSteps();
    bool getIsRunning();

  private:
    int _pin1;
    int _pin2;
    int _pin3;
    int _pin4;
    int _dir;
    int _currentSteps;
    int _targetSteps;
    unsigned long _lastStepTime;
    unsigned long _stepDelay;
    bool _isRunning;

    const int _steps[4][4] = {
        {HIGH, LOW, HIGH, LOW},  // Step 1
        {LOW, HIGH, HIGH, LOW},  // Step 2
        {LOW, HIGH, LOW, HIGH},  // Step 3
        {HIGH, LOW, LOW, HIGH},  // Step 4
    };

    const int _reversedSteps[4][4] = {
        {HIGH, LOW, LOW, HIGH},  // Step 4
        {LOW, HIGH, LOW, HIGH},  // Step 3
        {LOW, HIGH, HIGH, LOW},  // Step 2
        {HIGH, LOW, HIGH, LOW},  // Step 1
    };

    void setPins(const int states[4]);
};

#endif