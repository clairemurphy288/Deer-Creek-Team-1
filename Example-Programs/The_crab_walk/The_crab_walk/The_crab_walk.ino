// Let Mega walk like a crab


// generated by mBlock5 for MegaPi
// codes make you happy

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "src/MeSingleLineFollower.h"
#include "src/MeCollisionSensor.h"
#include "src/MeBarrierSensor.h"
#include "src/MeNewRGBLed.h"
#include <MeMegaPi.h>

MeBarrierSensor barrier_61(61);
MeMegaPiDCMotor motor_1(1);
MeMegaPiDCMotor motor_9(9);
MeMegaPiDCMotor motor_2(2);
MeMegaPiDCMotor motor_10(10);
MeMegaPiDCMotor motor_3(3);
MeMegaPiDCMotor motor_11(11);
MeMegaPiDCMotor motor_4(4);
MeMegaPiDCMotor motor_12(12);
double angle_rad = PI/180.0;
double angle_deg = 180.0/PI;

float T = 0;

void motor_foward_left_run(int16_t speed)
{
   motor_10.run(-speed);
}

void motor_foward_right_run(int16_t speed)
{
  motor_1.run(speed);
}

void motor_back_left_run(int16_t speed)
{
  motor_2.run(-speed);
}

void motor_back_right_run(int16_t speed)
{
  motor_9.run(speed);
}

void move_control(int16_t vx, int16_t vy, int16_t vw)
{
  int16_t foward_left_speed;
  int16_t foward_right_speed;
  int16_t back_left_speed;
  int16_t back_right_speed;

  foward_left_speed = vy + vx + vw;
  foward_right_speed = vy - vx - vw;
  back_left_speed = vy - vx + vw;
  back_right_speed = vy + vx - vw;

  motor_foward_left_run(foward_left_speed);
  motor_foward_right_run(foward_right_speed);
  motor_back_left_run(back_left_speed);
  motor_back_right_run(back_right_speed);
}

void _delay(float seconds) {
  if(seconds < 0.0){
    seconds = 0.0;
  }
  long endTime = millis() + seconds * 1000;
  while(millis() < endTime) _loop();
}

void setup() {
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  T = 0.7;

}

void _loop() {
}

void loop() {
  if(barrier_61.isBarried()){

      move_control(50 / 100.0 * 255,0 / 100.0 * 255,0 / 100.0 * 255);
      _delay(0.7);

      motor_1.run(0);
      motor_9.run(0);
      motor_2.run(0);
      motor_10.run(0);
      motor_3.run(0);
      motor_11.run(0);
      motor_4.run(0);
      motor_12.run(0);
      _delay(float(T));

      move_control(-50 / 100.0 * 255,50 / 100.0 * 255,0 / 100.0 * 255);
      _delay(0.7);

      motor_1.run(0);
      motor_9.run(0);
      motor_2.run(0);
      motor_10.run(0);
      motor_3.run(0);
      motor_11.run(0);
      motor_4.run(0);
      motor_12.run(0);
      _delay(float(T));

      move_control(50 / 100.0 * 255,50 / 100.0 * 255,0 / 100.0 * 255);
      _delay(0.7);

      motor_1.run(0);
      motor_9.run(0);
      motor_2.run(0);
      motor_10.run(0);
      motor_3.run(0);
      motor_11.run(0);
      motor_4.run(0);
      motor_12.run(0);
      _delay(float(T));

      move_control(-50 / 100.0 * 255,50 / 100.0 * 255,0 / 100.0 * 255);
      _delay(0.7);

      motor_1.run(0);
      motor_9.run(0);
      motor_2.run(0);
      motor_10.run(0);
      motor_3.run(0);
      motor_11.run(0);
      motor_4.run(0);
      motor_12.run(0);
      _delay(float(T));

      move_control(50 / 100.0 * 255,50 / 100.0 * 255,0 / 100.0 * 255);
      _delay(0.7);

      motor_1.run(0);
      motor_9.run(0);
      motor_2.run(0);
      motor_10.run(0);
      motor_3.run(0);
      motor_11.run(0);
      motor_4.run(0);
      motor_12.run(0);
      _delay(float(T));

      move_control(-50 / 100.0 * 255,0 / 100.0 * 255,0 / 100.0 * 255);
      _delay(0.7);

      motor_1.run(0);
      motor_9.run(0);
      motor_2.run(0);
      motor_10.run(0);
      motor_3.run(0);
      motor_11.run(0);
      motor_4.run(0);
      motor_12.run(0);
      _delay(float(T));

      move_control(50 / 100.0 * 255,-50 / 100.0 * 255,0 / 100.0 * 255);
      _delay(0.7);

      motor_1.run(0);
      motor_9.run(0);
      motor_2.run(0);
      motor_10.run(0);
      motor_3.run(0);
      motor_11.run(0);
      motor_4.run(0);
      motor_12.run(0);
      _delay(float(T));

      move_control(-50 / 100.0 * 255,-50 / 100.0 * 255,0 / 100.0 * 255);
      _delay(0.7);

      motor_1.run(0);
      motor_9.run(0);
      motor_2.run(0);
      motor_10.run(0);
      motor_3.run(0);
      motor_11.run(0);
      motor_4.run(0);
      motor_12.run(0);
      _delay(float(T));

      move_control(50 / 100.0 * 255,-50 / 100.0 * 255,0 / 100.0 * 255);
      _delay(0.7);

      motor_1.run(0);
      motor_9.run(0);
      motor_2.run(0);
      motor_10.run(0);
      motor_3.run(0);
      motor_11.run(0);
      motor_4.run(0);
      motor_12.run(0);
      _delay(float(T));

      move_control(-50 / 100.0 * 255,-50 / 100.0 * 255,0 / 100.0 * 255);
      _delay(0.7);

      motor_1.run(0);
      motor_9.run(0);
      motor_2.run(0);
      motor_10.run(0);
      motor_3.run(0);
      motor_11.run(0);
      motor_4.run(0);
      motor_12.run(0);

  }

  _loop();
}
