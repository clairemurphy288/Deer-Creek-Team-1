//The three infrared obstacle avoidance sensors on the left, center and right respectively represent left rotation, stop, and right rotation. 


#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "src/MeSingleLineFollower.h"
#include "src/MeCollisionSensor.h"
#include "src/MeBarrierSensor.h"
#include "src/MeNewRGBLed.h"
#include <MeMegaPi.h>

MeBarrierSensor barrier_60(60);
MeMegaPiDCMotor motor_1(1);
MeMegaPiDCMotor motor_9(9);
MeBarrierSensor barrier_61(61);
MeMegaPiDCMotor motor_2(2);
MeMegaPiDCMotor motor_10(10);
MeMegaPiDCMotor motor_3(3);
MeMegaPiDCMotor motor_11(11);
MeMegaPiDCMotor motor_4(4);
MeMegaPiDCMotor motor_12(12);
MeBarrierSensor barrier_62(62);
double angle_rad = PI/180.0;
double angle_deg = 180.0/PI;

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

}

void _loop() {
}

void loop() {
  if(barrier_60.isBarried()){

      motor_1.run(50 / 100.0 * 255);
      motor_9.run(50 / 100.0 * 255);

  }
  if(barrier_61.isBarried()){

      motor_1.run(0);
      motor_9.run(0);
      motor_2.run(0);
      motor_10.run(0);
      motor_3.run(0);
      motor_11.run(0);
      motor_4.run(0);
      motor_12.run(0);

  }
  if(barrier_62.isBarried()){

      motor_2.run(50 / 100.0 * 255);
      motor_10.run(50 / 100.0 * 255);

  }

  _loop();
}
