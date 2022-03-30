/*
 BraccioV3.cpp
   Refactored to replace servo library with Adafruit-PWM-Servo-Driver-Library
   Notes: 
    - servo library is interrupt driven, Adafruit-PWM-Servo-Driver-Library is not. 
    - servo::write() sets the servo angle in degrees 0-180, while Adafruit-PWM-Servo-Driver-Library::setPin sets PWM frequency from 0-4095
   modified by Rowan patterson 2021 from: 
 BraccioV2.cpp - version 0.1
    Written by Lukas Severinghaus
 Based upon the Braccio library by Andrea Martino and Angelo Ferrante
    originally licensed under GNU GPL V1.2.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "BraccioV3.h"
Braccio::Braccio() {
}

void Braccio::begin() {
  _initializeServos(true);
}

void Braccio::begin(bool defaultPos) {
  _initializeServos(defaultPos);
}

//Initializes all servos by attaching and optionally moving them all
void Braccio::_initializeServos(bool defaultPos) {
  if (defaultPos) {
    setAllNow(_jointCenter[BASE_ROT], _jointCenter[SHOULDER], _jointCenter[ELBOW],
              _jointCenter[WRIST], _jointCenter[WRIST_ROT], _jointCenter[GRIPPER]);

  }
}

/*
   Sets one joint to absolute target position. Constrains value to min/max limits,
   returns true if value was not constrained, false if it was constrained to the limits.
*/
bool Braccio::setOneAbsolute(int joint, int value) {
  int out = constrain(value, _jointMin[joint], _jointMax[joint]);
  targetJointPositions[joint] = out;
  return joint == out;
}

/*
   Sets one joint to a target position relative to current target position. Constrains
   value to min/max limits, returns true if value was not constrained, false if it was
   constrained to limits.
*/
bool Braccio::setOneRelative(int joint, int value) {
  int currentPos = targetJointPositions[joint];
  int rawPos = currentPos + value;
  int actualPos = constrain(rawPos, _jointMin[joint], _jointMax[joint]);
  targetJointPositions[joint] = actualPos;
  return rawPos == actualPos;
}

/*
   Sets all joints to absolute target positions. Returns true if none of the values
   were constrained, false if at least one of the values was constrained to limits.
*/
bool Braccio::setAllAbsolute(int b, int s, int e, int w, int w_r, int g) {
  boolean out = true;
  out = out & setOneAbsolute(BASE_ROT, b);
  out = out & setOneAbsolute(SHOULDER, s);
  out = out & setOneAbsolute(ELBOW, e);
  out = out & setOneAbsolute(WRIST, w);
  out = out & setOneAbsolute(WRIST_ROT, w_r);
  out = out & setOneAbsolute(GRIPPER, g);
  return out;
}

/*
   Sets all joints to relative target positions. Returns true if none of the values
   were constrained, false if at least one of the values was constrained to limits.
*/
bool Braccio::setAllRelative(int b, int s, int e, int w, int w_r, int g) {
  boolean out = true;
  out = out & setOneRelative(BASE_ROT, b);
  out = out & setOneRelative(SHOULDER, s);
  out = out & setOneRelative(ELBOW, e);
  out = out & setOneRelative(WRIST, w);
  out = out & setOneRelative(WRIST_ROT, w_r);
  out = out & setOneRelative(GRIPPER, g);
  return out;
}

//Sets maximum command value of given joint
void Braccio::setJointMax(int joint, int value) {
  _jointMax[joint] = constrain(value, GLOBAL_MIN, GLOBAL_MAX);
}

//Sets minimum command value of given joint
void Braccio::setJointMin(int joint, int value) {
  _jointMin[joint] = constrain(value, GLOBAL_MIN, GLOBAL_MAX);
}

//Set the center point of given joint
void Braccio::setJointCenter(int joint, int offset) {
  _jointCenter[joint] = constrain(offset, GLOBAL_MIN, GLOBAL_MAX);
}

/*
   Sets all joints to absolute position immediately. Does not constrain values.
*/
void Braccio::setAllNow(int b, int s, int e, int w, int w_r, int g) {
  _setServo(BASE_ROT, b, true);
  _setServo(SHOULDER, s, true);
  _setServo(ELBOW, e, true);
  _setServo(WRIST, w, true);
  _setServo(WRIST_ROT, w_r, true);
  _setServo(GRIPPER, g, true);
}

//Sets the speed of a given joint, defaults to 1
void Braccio::setDelta(int joint, int value) {
  _jointDelta[joint] = value;
}

//Sets a given joint to a specific position
void Braccio::_setServo(int joint, int value, bool updateTarget) {
  int pwm_duty_cycle = map(value, 0, 180, 0, 4096);
  switch (joint) {
    case BASE_ROT:
      _pwm.setPin(_BASE_ROT_PIN, pwm_duty_cycle, false);
      currentJointPositions[BASE_ROT] = value;
      if (updateTarget) {
        targetJointPositions[BASE_ROT] = value;
      }
      break;
    case SHOULDER:
      _pwm.setPin(_SHOULDER_PIN, pwm_duty_cycle, false);
      currentJointPositions[SHOULDER] = value;
      if (updateTarget) {
        targetJointPositions[SHOULDER] = value;
      }
      break;
    case ELBOW:
      _pwm.setPin(_ELBOW_PIN, pwm_duty_cycle, false);
      currentJointPositions[ELBOW] = value;
      if (updateTarget) {
        targetJointPositions[ELBOW] = value;
      }
      break;
    case WRIST:
      _pwm.setPin(_WRIST_PIN, pwm_duty_cycle, false);
      currentJointPositions[WRIST] = value;
      if (updateTarget) {
        targetJointPositions[WRIST] = value;
      }
      break;
    case WRIST_ROT:
      _pwm.setPin(_WRIST_ROT_PIN, pwm_duty_cycle, false);
      currentJointPositions[WRIST_ROT] = value;
      if (updateTarget) {
        targetJointPositions[WRIST_ROT] = value;
      }
      break;
    case GRIPPER:
      _pwm.setPin(_GRIPPER_PIN, pwm_duty_cycle, false);
      currentJointPositions[GRIPPER] = value;
      if (updateTarget) {
        targetJointPositions[GRIPPER] = value;
      }
      break;
  }
}

//Processes servo movement request
void Braccio::_moveServo(int joint) {
  int currentPos = currentJointPositions[joint];
  int targetPos = targetJointPositions[joint];
  if (currentPos != targetPos) {
    int dir = (currentPos <= targetPos) ? 1 : -1;
    int delta = _jointDelta[joint];
    int dirDelta = dir * delta;
    int newPos = currentPos + dirDelta;
    _setServo(joint, newPos, false);
  }
}

//Delays for ms with movement updates every t ms
void Braccio::safeDelay(int ms, int t){
  long currentTime = millis();
  while(millis() < currentTime + ms){
    update();
    delay(t);
  }
}

//Delays with movement updates every 10ms
void Braccio::safeDelay(int ms){
  safeDelay(ms, 10);
}
//Returns the calibrated center point of the given joint
int Braccio::getCenter(int joint){
  return _jointCenter[joint];
}
//Processes movement of each joint to achieve target endpoint
void Braccio::update() {
  _moveServo(BASE_ROT);
  _moveServo(SHOULDER);
  _moveServo(ELBOW);
  _moveServo(WRIST);
  _moveServo(WRIST_ROT);
  _moveServo(GRIPPER);

}
