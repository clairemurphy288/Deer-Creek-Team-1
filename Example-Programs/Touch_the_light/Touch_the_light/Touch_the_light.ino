// Touching the A9 patrol sensor causes RGB to light one light until all eight of the Mega's lights light up, then again starting with the first light.


#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "src/MeSingleLineFollower.h"
#include "src/MeCollisionSensor.h"
#include "src/MeBarrierSensor.h"
#include "src/MeNewRGBLed.h"
#include <MeMegaPi.h>

MeNewRGBLed rgbled_67(67,4);
MeNewRGBLed rgbled_68(68,4);
MeSingleLineFollower linefollower_63(63);
double angle_rad = PI/180.0;
double angle_deg = 180.0/PI;

float line_sta = 0;
float line_last = 0;
float light_num = 0;

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
  light_num = 0;

  rgbled_67.setColor(0, 0, 0, 0);
  rgbled_67.show();

  rgbled_68.setColor(0, 0, 0, 0);
  rgbled_68.show();

}

void _loop() {
}

void loop() {
  if((linefollower_63.readSensor() == 1)){
      if(line_last == line_sta){
          line_sta += 1;
          light_num += 1;
          if(light_num > 8){
              light_num = 1;

              rgbled_67.setColor(0, 0, 0, 0);
              rgbled_67.show();

              rgbled_68.setColor(0, 0, 0, 0);
              rgbled_68.show();

          }
          if(light_num > 4){

              rgbled_68.setColor((light_num - 4), 211,32,32);
              rgbled_68.show();

          }else{

              rgbled_67.setColor(light_num, 126,211,33);
              rgbled_67.show();

          }

      }

  }else{
      line_last = line_sta;

  }

  _loop();
}
