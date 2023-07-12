

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "MeSingleLineFollower.h"
#include "MeCollisionSensor.h"
#include "MeBarrierSensor.h"
#include "MeNewRGBLed.h"
#include <MeMegaPi.h>

MeNewRGBLed rgbled_67(67,4);
MeNewRGBLed rgbled_68(68,4);
MeSingleLineFollower linefollower_63(63);
MeSingleLineFollower linefollower_64(64);
MeBarrierSensor barrier_61(61);
MeBarrierSensor barrier_62(62);
//MeBarrierSensor barrier_63(63);
MeMegaPiDCMotor motor_1(1);
MeMegaPiDCMotor motor_9(9);
MeMegaPiDCMotor motor_2(2);
MeMegaPiDCMotor motor_10(10);

double angle_rad = PI/180.0;
double angle_deg = 180.0/PI;

float T = 0;
double power = 0.5;

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
  // put your setup code here, to run once:
  Serial.begin(9600); 
  rgbled_67.fillPixelsBak(0, 2, 1);
  rgbled_68.fillPixelsBak(0, 2, 1);
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  T = 0.7;
  rgbled_67.setColor(0, 0, 0, 0);
  rgbled_67.show();

  rgbled_68.setColor(0, 0, 0, 0);
  rgbled_68.show();
}
void _loop() {
}
void lineFollow(){
  //for (int i =0; i<4; i++){
  //rgbled_67.setColor(i, 100, 100, 100);
  //rgbled_67.show();

  //rgbled_68.setColor(i, 100, 100, 100);
  //rgbled_68.show();
  //}
  while (linefollower_63.readSensor() !=0 && linefollower_64.readSensor !=0) {

    if (linefollower_63.readSensor()==1 && linefollower_64.readSensor()==1) {
      Forward();
    Serial.println("both"); 
    }

  else if (linefollower_63.readSensor()==1 && linefollower_64.readSensor()==0){
    TurnRight();
    Serial.println("right"); 
  }

  else if (linefollower_63.readSensor()==0 && linefollower_64.readSensor()==1){
    TurnLeft();
    Serial.println("left") ;
  }

  else (linefollower_63.readSensor()==0 && linefollower_64.readSensor()==0){
    Stop();
    Serial.println("neither");
  }
  
}

void ObstacleAvoidance(){
  Backward();
  _delay(0.5);
  TurnRight();
  _delay(0.5);
  while(linefollower_63.readSensor()==0 && linefollower_64.readSensor()==0) {
    motor_1.run(-0.3*255);
    motor_9.run(-0.3*255);
    motor_2.run(0.5*255);
    motor_10.run(0.5*255);
  }
  }

void Forward(){
  motor_1.run(-power*255);
  motor_9.run(-power*255);
  motor_2.run(power*255);
  motor_10.run(power*255);
  }

void Backward(){
  motor_1.run(power*255);
  motor_9.run(power*255);
  motor_2.run(-power*255);
  motor_10.run(-power*255);
 
}

void TurnLeft(){
  motor_1.run(0);
  motor_9.run(0);
  motor_2.run(power*255);
  motor_10.run(power*255);
  
}

void TurnRight(){
  motor_1.run(-power*255);
  motor_9.run(-power*255);
  motor_2.run(0);
  motor_10.run(0);
  
}

void StrafeLeft(){

move_control(power * 255,0 / 100.0 * 255,0 / 100.0 * 255);
_delay(float(T));
}

void StrafeRight(){
move_control(-power * 255,0 / 100.0 * 255,0 / 100.0 * 255);
_delay(float(T));
}

void Stop(){
  motor_1.run(0);
  motor_9.run(0);
  motor_2.run(0);
  motor_10.run(0);
}
void loop() {
  // put your main code here, to run repeatedly:
    if (barrier_61.isBarried() || barrier_62.isBarried()) {
      ObstacleAvoidance();
    }
    else {
    lineFollow();
  }  
}



// struct State {
//   private:
//     int wheel1Speed; 
//     int wheel2Speed; 
//     int wheel3Speed; 
//     int wheel4Speed;
//     int timeInState; 
// }

// State states[15000]; 

// void traceMemorizedLine() { 
//   boolean keepGoing = true; 
//   int i = 0; 
//   while (keepGoing) {
//     if (i == (sizeof(states)/sizeof(states[0]))) {
//       keepGoing = false; 
//     }
//     if (states[i] == 0) {
//       keepGoing = false; 
//     }

//     State* currState = states[i]; 
//     // then get stuff for each wheel and call functions to
//   }
// }





