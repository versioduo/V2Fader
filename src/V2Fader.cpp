// © Kay Sievers <kay@versioduo.com>, 2020-2022
// SPDX-License-Identifier: Apache-2.0

#include "V2Fader.h"

void V2Fader::begin() {
  pinMode(_pin.motor.enable, OUTPUT);
  digitalWrite(_pin.motor.enable, LOW);
  pinMode(_pin.motor.phase, OUTPUT);
  digitalWrite(_pin.motor.phase, LOW);
  pinMode(_pin.motor.sleep, OUTPUT);
  digitalWrite(_pin.motor.sleep, HIGH);

  // Take over the entire ADC, it can no longer be shared and used with multiple channels/pins.
  // It will sample this single channel continuously. The result is read in interrupt context.
  // Other pins and analogRead() can no longer be used.
  _adc = new V2Base::Analog::ADC(V2Base::Analog::ADC::getID(_pin.wiper));
  _adc->begin();
  _adc->sampleChannel(V2Base::Analog::ADC::getChannel(_pin.wiper));
}

void V2Fader::reset() {
  _state    = State::Idle;
  speed_max = 0.5;
  _move     = {};
  setSpeed(0);
}

void V2Fader::loop() {
  if ((unsigned long)(micros() - _usec) < 5000)
    return;
  _usec = micros();

  switch (_state) {
    case State::Move:
      // Serial.print("   position=");
      // Serial.print(getFraction(), 3);
      // Serial.print(" delta=");
      // Serial.print(_move.target - getFraction(), 3);
      // Serial.print(" duty=");
      // Serial.print(_move.duty, 3);
      // Serial.print(" current=");
      // Serial.println(_move.current.ampere, 3);
      break;

    case State::Stop:
      // Serial.print("E: time=");
      // Serial.print((micros() - _move.usec) / 1000);
      // Serial.print(" position=");
      // Serial.println(getFraction(), 3);
      _state = State::Idle;
      break;
  }
}

void V2Fader::move(float target) {
  // Serial.print("S: position=");
  // Serial.print(getFraction(), 3);
  // Serial.print(" target=");
  // Serial.println(target, 3);

  _move = {.usec{micros()}, .start{getFraction()}, .target{target}};
  digitalWrite(_pin.motor.phase, _move.start > _move.target);
  _state = State::Move;
}

void V2Fader::stop() {
  if (_state != State::Move)
    return;

  setSpeed(0);
  _state = State::Stop;
}

void V2Fader::tick() {
  measure(1.f - _adc->read());

  switch (_state) {
    case State::Move: {
      // Limit the time of the movement, something might be in our
      // way and we never reach the target.
      if ((unsigned long)(micros() - _move.usec) > 2 * 1000 * 1000) {
        setSpeed(0);
        _state = State::Stop;
        break;
      }

      // Are we close enough? The DC motor needs to start moving, it cannot
      // travel very small distances.
      if (fabs(_move.target - getFraction()) < config->lag) {
        setSpeed(0);
        _state = State::Stop;
        break;
      }

      // Do not move back. Ignore overshooting, or if something else moved us
      // farther than the target.
      if ((_move.target > _move.start) == (getFraction() > _move.target)) {
        setSpeed(0);
        _state = State::Stop;
        break;
      }

      // Adjust the speed proportional to the remaining distance to move.
      const float speed = fabs(_move.target - getFraction()) * powf(speed_max, 3);

      // Softer start.
      if (_move.speed < speed)
        _move.speed += 0.03f;
      else
        _move.speed = speed;

      // Gradually increase the minimum speed until we start moving.
      if (_move.minimum < 0.2f && fabs(_move.start - getFraction()) < 0.01f)
        _move.minimum += 0.01f;

      const float minimum = _speed.min + _move.minimum;
      _move.duty          = minimum + ((_speed.max - minimum) * _move.speed);
      setSpeed(_move.duty);
    } break;

    case State::Stop:
      break;
  }
}

void V2Fader::setSpeed(float duty) {
  _pwm->setDuty(_pin.motor.enable, duty);
}

// Motor resistance ~17.5Ω, 12V → 0.68A
void V2Fader::measureCurrent() {
  const float fraction = (float)analogRead(_pin.motor.current) / 1024.f;

  // Low-pass filter, smooth the value.
  const float alpha = 0.3;
  _move.current.analog *= 1.f - alpha;
  _move.current.analog += fraction * alpha;

  // Motor driver current sense: 3.3V/2.7Ω → 1.22A
  _move.current.ampere = _move.current.analog * 1.22f;
}
