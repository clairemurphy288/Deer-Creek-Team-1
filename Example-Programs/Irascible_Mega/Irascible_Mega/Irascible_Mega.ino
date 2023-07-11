//Pick up the four wheels of Mega and turn them randomly, and the two RGB lights will go out.
//Put down the four wheels of Mega and stop, and the RGB lights will turn on red. 


#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "src/MeSingleLineFollower.h"
#include "src/MeCollisionSensor.h"
#include "src/MeBarrierSensor.h"
#include "src/MeNewRGBLed.h"
#include <MeMegaPi.h>

MeSingleLineFollower linefollower_63(63);
MeSingleLineFollower linefollower_64(64);
MeNewRGBLed rgbled_67(67,4);
MeNewRGBLed rgbled_68(68,4);
MeMegaPiDCMotor motor_1(1);
MeMegaPiDCMotor motor_2(2);
MeMegaPiDCMotor motor_9(9);
MeMegaPiDCMotor motor_10(10);
MeMegaPiDCMotor motor_3(3);
MeMegaPiDCMotor motor_11(11);
MeMegaPiDCMotor motor_4(4);
MeMegaPiDCMotor motor_12(12);
double angle_rad = PI/180.0;
double angle_deg = 180.0/PI;

float RGB = 0;

void _delay(float seconds) {
  if(seconds < 0.0){
    seconds = 0.0;
  }
  long endTime = millis() + seconds * 1000;
  while(millis() < endTime) _loop();
}

void setup() {
  rgbled_67.fillPixelsBak(0, 2, 1);
  rgbled_68.fillPixelsBak(0, 2, 1);
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

}

void _loop() {
}

void loop() {
  if(((linefollower_63.readSensor() == 0))  &&  ((linefollower_64.readSensor() == 0))){

      rgbled_67.setColor(0, 0, 0, 0);
      rgbled_67.show();

      rgbled_68.setColor(0, 0, 0, 0);
      rgbled_68.show();

      motor_1.run(50 / 100.0 * 255);
      motor_2.run(50 / 100.0 * 255);

      motor_9.run(-50 / 100.0 * 255);
      motor_10.run(-50 / 100.0 * 255);

  }
  if(((linefollower_63.readSensor() == 1))  &&  ((linefollower_64.readSensor() == 1))){

      rgbled_67.setColor(0, 255, 0, 0);
      rgbled_67.show();

      rgbled_68.setColor(0, 255, 0, 0);
      rgbled_68.show();

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
