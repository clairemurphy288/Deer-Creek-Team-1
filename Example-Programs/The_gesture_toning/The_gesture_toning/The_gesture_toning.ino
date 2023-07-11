//Determine whether there is an obstacle in front. If the left sensor detects the obstacle, RGB displays a green light, 
//the middle sensor detects an obstacle RGB displays a yellow light, and the right sensor detects an obstacle RGB displays a red light warning.


#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "src/MeSingleLineFollower.h"
#include "src/MeCollisionSensor.h"
#include "src/MeBarrierSensor.h"
#include "src/MeNewRGBLed.h"
#include <MeMegaPi.h>

MeBarrierSensor barrier_60(60);
MeNewRGBLed rgbled_67(67,4);
MeNewRGBLed rgbled_68(68,4);
MeBarrierSensor barrier_61(61);
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
  if(barrier_60.isBarried()){

      rgbled_67.setColor(0, 126,211,33);
      rgbled_67.show();

      rgbled_68.setColor(0, 126,211,33);
      rgbled_68.show();

  }
  if(barrier_61.isBarried()){

      rgbled_67.setColor(0, 255, 255, 0);
      rgbled_67.show();

      rgbled_68.setColor(0, 255, 255, 0);
      rgbled_68.show();

  }
  if(barrier_62.isBarried()){

      rgbled_67.setColor(0, 255, 0, 0);
      rgbled_67.show();

      rgbled_68.setColor(0, 255, 0, 0);
      rgbled_68.show();

  }

  _loop();
}
