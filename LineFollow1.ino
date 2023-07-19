

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
  double startTime = millis();  // 5 
  int n = -1; 

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







/* LINE MEMORIZATION */ 

/* 
 * Structure to store a state of the robot 
 * and the duration in that state  
 */
struct State {
  double motor1Power; 
  double motor9Power; 
  double motor2Power;  
  double motor10Power;  
  double time; 
}

// Enum for the line state 
enum lineState { forward, left, right, stop, start}; 

int numStates = 0; // total states we go through 
State states[1000]; // stores the memorized path 

/*
 * Does line following, while also 
 * storing the path followed
 */ 
void memorizeLine() {
  double startTime = millis(); 
  lineState currState = start; 
  int index = 0;

  while (linefollower_63.readSensor() !=0 && linefollower_64.readSensor !=0) {

    // forward
    if (linefollower_63.readSensor()==1 && linefollower_64.readSensor()==1) {
      Forward();
      if (currState != forward) {
          double t = millis() - startTime; 
          double p = power * 255; 
          State temp = {-p, -p, p, p, t};
          states[index] = temp; 
          index += 1; 
          startTime = millis(); 
          currState = forward;  
      }
      Serial.println("in state: forward"); 
    }

  // right 
  else if (linefollower_63.readSensor()==1 && linefollower_64.readSensor()==0){
    TurnRight();
    if (currState != right) {
          double t = millis() - startTime;  
          State temp = {-power*255, -power*255, 0, 0, t};
          states[index] = temp; 
          index += 1; 
          startTime = millis(); 
          currState = right;  
    }
    Serial.println("in state: right"); 
  }

  // left 
  else if (linefollower_63.readSensor()==0 && linefollower_64.readSensor()==1){
    TurnLeft();
    if (currState != left) {
          double t = millis() - startTime;  
          State temp = {0, 0, power*255, power*255, t}; 
          states[index] = temp; 
          index += 1; 
          startTime = millis(); 
          currState = left;  
    }
    Serial.println("in state: left") ;
  }

  // stop 
  else (linefollower_63.readSensor()==0 && linefollower_64.readSensor()==0){
    Stop();
    if (currState != stop) {
          State temp = {0, 0, 0, 0, 0}
          states[index] = temp; 
          numStates = index + 1; 
          return states; 
    }
    Serial.println("neither");
  }  
} 

/* 
 * Retraces a memorized line given an array of States 
 * and the number of states in the array 
 */ 
void retraceLine(State states[], int n) {
  for (int i = 0; i < n; i++) {
    State currState = states[i];
    motor1_.run(currState.motor1Power);
    motor9_.run(currState.motor9Power); 
    motor_2.run(currState.motor2Power);
    motor_10.run(currState.motor10Power);   
    _delay(currState.time); 
  }
}


/* Main code for line memorization and retracing */ 
void lineMemorizeMain() {
  memorizeLine(); 
  // do something for delay while moving robot 
  retraceLine(states, numStates); 
}