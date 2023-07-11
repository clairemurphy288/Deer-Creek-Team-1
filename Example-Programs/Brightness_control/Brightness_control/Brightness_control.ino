//Block the 3 infrared obstacle avoidance sensors, the R\G\B value of the RGB lamp module will change, 
//and RGB corresponds to the brightness of the RGB according to the order of the infrared obstacle avoidance sensors on the left, middle and right.


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

float light_1 = 0;
float light_2 = 0;
float light_3 = 0;

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
  light_1 = random(0, 80 +1);
  light_2 = random(80, 150 +1);
  light_3 = random(150, 255 +1);

}

void _loop() {
}

void loop() {
  if(barrier_60.isBarried()){

      rgbled_67.setColor(0, light_1, light_1, light_1);
      rgbled_67.show();

      rgbled_68.setColor(0, light_1, light_1, light_1);
      rgbled_68.show();

  }
  if(barrier_61.isBarried()){

      rgbled_67.setColor(0, light_2, light_2, light_2);
      rgbled_67.show();

      rgbled_68.setColor(0, light_2, light_2, light_2);
      rgbled_68.show();

  }
  if(barrier_62.isBarried()){

      rgbled_67.setColor(0, light_3, light_3, light_3);
      rgbled_67.show();

      rgbled_68.setColor(0, light_3, light_3, light_3);
      rgbled_68.show();

  }

  _loop();
}
