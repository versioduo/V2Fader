// © Kay Sievers <kay@versioduo.com>, 2020-2022
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <V2Base.h>
#include <V2Potentiometer.h>

class V2Fader : public V2Potentiometer {
public:
  V2Fader(uint8_t pinWiper,
          uint8_t pinTouch,
          uint8_t pinMotorEnable,
          uint8_t pinMotorPhase,
          uint8_t pinMotorSleep,
          uint8_t pinMotorCurrent,
          V2Base::Timer::PWM *pwm) :
    V2Potentiometer(&_config),
    _pin{.wiper{pinWiper},
         .touch{pinTouch},
         .motor{
           .enable{pinMotorEnable},
           .phase{pinMotorPhase},
           .sleep{pinMotorSleep},
           .current{pinMotorCurrent},
         }},
    _pwm{pwm} {}

  float speed_max{0.5};

  void begin();
  void reset();
  void loop();

  // Move/stop the fader.
  void move(float target);
  void stop();

  bool isBusy() {
    return _state != State::Idle;
  }

  // Timer tick, called ~500 times / second. Reads the analog port and adjusts
  // the motor speed to drive toward the target position.
  void tick();

private:
  const V2Potentiometer::Config _config{
    .n_steps{128},
    .min{0.02},
    .max{0.98},
    .alpha{0.5},
    .lag{0.007},
  };

  const struct {
    uint8_t wiper;
    uint8_t touch;
    struct {
      uint8_t enable;
      uint8_t phase;
      uint8_t sleep;
      uint8_t current;
    } motor;
  } _pin;
  V2Base::Timer::PWM *_pwm;

  volatile enum class State { Idle, Move, Stop } _state{};
  unsigned long _usec{};

  static constexpr struct {
    float min;
    float max;
  } _speed{.min{0.1}, .max{0.7}};

  struct {
    unsigned long usec;
    float start;
    float target;
    float minimum;
    float speed;
    float duty;
    struct {
      float analog;
      float ampere;
    } current;
  } _move{};

  V2Base::Analog::ADC *_adc{};

  void setSpeed(float duty);
  void measureCurrent();
};
