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
MeCollisionSensor collision_65(65);
MeCollisionSensor collision_66(66);

// Enum for the line state 
enum lineState { forward, left, right, stop, start}; 

// Struct to store a state while line following 
struct  State{
  lineState state; 
  double time; 
};

// Used in Line Following for sharp turns 
lineState prevTurn = forward; 

double angle_rad = PI/180.0; 
double angle_deg = 180.0/PI; 

float T = 0;
double power = 0.3;



/* ----------------------------------------------------------------------------------------------- */
/* USEFUL FUNCTIONS 
/* ----------------------------------------------------------------------------------------------- */



void motor_foward_left_run(int16_t speed) {
   motor_10.run(-speed);
}

void motor_foward_right_run(int16_t speed) {
  motor_1.run(speed);
}

void motor_back_left_run(int16_t speed) {
  motor_2.run(-speed);
}

void motor_back_right_run(int16_t speed) {
  motor_9.run(speed);
}

void move_control(int16_t vx, int16_t vy, int16_t vw) {
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

void Forward() {
  motor_1.run(-power*255);
  motor_9.run(-power*255);
  motor_2.run(power*255);
  motor_10.run(power*255);
}

void Backward() {
  motor_1.run(power*255);
  motor_9.run(power*255);
  motor_2.run(-power*255);
  motor_10.run(-power*255);
}

void TurnLeft() {
  motor_1.run(power*255);
  motor_9.run(power*255);
  motor_2.run(power*255);
  motor_10.run(power*255);
}

void TurnRight() {
  motor_1.run(-power*255);
  motor_9.run(-power*255);
  motor_2.run(-power*255);
  motor_10.run(-power*255);
}

void StrafeLeft() {
  move_control(power * 255,0 / 100.0 * 255,0 / 100.0 * 255);
  _delay(float(T));
}

void StrafeRight() {
  move_control(-power * 255,0 / 100.0 * 255,0 / 100.0 * 255);
  _delay(float(T));
}

void Stop() {
  motor_1.run(0);
  motor_9.run(0);
  motor_2.run(0);
  motor_10.run(0);
}



/* ----------------------------------------------------------------------------------------------- */
/* LINE FOLLOWING
/* ----------------------------------------------------------------------------------------------- */



/* 
 * Follows a line 
 * Takes a boolean for whether or not we are avoiding obstacles
 */ 
void lineFollow() {
  // Turn on lights 
  for (int i =0; i<4; i++){
    rgbled_67.setColor(i, 255, 255, 255);
    rgbled_67.show();
    rgbled_68.setColor(i, 255, 255, 255);
    rgbled_68.show();
  }

  // Stop when back left pressed
  while (!collision_66.isCollision()) {

    // Check if we are avoiding obstacles, if so check sensors 
    if (barrier_61.isBarried() || barrier_62.isBarried()) {
      ObstacleAvoidance(); 
    }

    // Forward 
    else if (linefollower_63.readSensor()==0 && linefollower_64.readSensor()==0) {
      Forward();
      //Serial.println("both"); 
    }

    // Right 
    else if (linefollower_63.readSensor()==1 && linefollower_64.readSensor()==0){
      TurnRight();
      prevTurn = right; 
      //Serial.println("right");  
    }

    // Left 
    else if (linefollower_63.readSensor()==0 && linefollower_64.readSensor()==1){
      TurnLeft();
      prevTurn = left; 
      //Serial.println("left") ;
    }

    // Off the line
    else {
      //Serial.println("neither"); 
      // Turn previous direction until back on line
      while (linefollower_63.readSensor()==1 && linefollower_64.readSensor()==1) {
        if (prevTurn == left) {
          TurnLeft(); 
        }
        else if (prevTurn == right) {
          TurnRight(); 
        }
      }
    }
  }

  // out of loop 
  Stop(); 
  // Turn off lights 
  for (int i =0; i<4; i++){
    rgbled_67.setColor(i, 0, 0, 0);
    rgbled_67.show();
    rgbled_68.setColor(i, 0, 0, 0);
    rgbled_68.show();
  }
}



/* ----------------------------------------------------------------------------------------------- */
/* OBSTACLE AVOIDANCE 
/* ----------------------------------------------------------------------------------------------- */



/* Avoids a single obstacle 
 * This function is called inside the main loop when an obstacle is encountered */
void ObstacleAvoidance() {
  Backward();
  _delay(0.5);
  TurnRight();
  _delay(0.8);
  while(linefollower_63.readSensor()==1 && linefollower_64.readSensor()==1) {
    motor_1.run(-0.3*255);
    motor_9.run(-0.3*255);
    motor_2.run(0.5*255);
    motor_10.run(0.5*255);
  }
}



/* ----------------------------------------------------------------------------------------------- */
/* LINE MEMORIZATION 
/* ----------------------------------------------------------------------------------------------- */



/*
 * Does line following, while also 
 * storing the path followed 
 */ 
void memorizeLine(State path[], int n) {
  Serial.println("MEMORIZING LINE"); 
  // Turn on lights 
  for (int i =0; i<4; i++){
    rgbled_67.setColor(i, 255, 255, 255);
    rgbled_67.show();
    rgbled_68.setColor(i, 255, 255, 255);
    rgbled_68.show();
  }
  
  double startTime = millis();
  lineState currState = start;
  int index = 0; 

  // Go until back right impact switch is pressed, or path array becomes full
  //while (!collision_66.isCollision()) {
  while (linefollower_63.readSensor()!=1 && linefollower_64.readSensor()!=1) {
    
    // Array is full, stop 
    if (index == n-1) {
      Serial.println("FULL ARRAY");
      break; 
    }

    // Forward
    else if (linefollower_63.readSensor()==0 && linefollower_64.readSensor()==0) {
      Forward();
      if (currState != forward) { // We came from another state 
        double t = millis() - startTime; // Get the time for the state we were in 
        path[index] = {currState, t}; // Store the state we were in 
        index = index + 1; 
        startTime = millis(); // Reset time to start tracking current state
        currState = forward; // Change state to state we are now in
      }
      //Serial.println("in state: forward"); 
    }

    // Right 
    else if (linefollower_63.readSensor()==1 && linefollower_64.readSensor()==0) {
      TurnRight();
      if (currState != right) {
        double t = millis() - startTime;  
        path[index] = {currState, t}; 
        index = index + 1; 
        startTime = millis();
        currState = right;  
      }
      //Serial.println("in state: right"); 
    }

    // Left 
    else if (linefollower_63.readSensor()==0 && linefollower_64.readSensor()==1) {
      TurnLeft();
      if (currState != left) {
        double t = millis() - startTime;  
        path[index] = {currState, t}; 
        index = index + 1; 
        startTime = millis(); 
        currState = left;  
      }
      //Serial.println("in state: left") ;
    }

    // Both off 
    else {
      // turn the direction we were last turning until back on line
      while (linefollower_63.readSensor()==1 && linefollower_64.readSensor()==1) {
        if (currState == left) {
          TurnLeft(); 
        }
        else if (currState == right) {
          TurnRight(); 
        }
      }
    }
  }

  // Out of while loop 
  Stop(); 
  path[index] = {currState, millis() - startTime}; // store the last state 
}


/* 
 * Retraces a memorized line given an array of States 
 * and the number of states in the array 
 */ 
void retraceLine(State states[], int n) {
  // Turn off lights 
  for (int i =0; i<4; i++){
    rgbled_67.setColor(i, 0, 0, 0);
    rgbled_67.show();
    rgbled_68.setColor(i, 0, 0, 0);
    rgbled_68.show();
  }
  
  Serial.println("RETRACING LINE"); 
  for (int i = 0; i < n; i++) {
    State currState = states[i];
    lineState s = currState.state; 
    if (s == forward) {
      Serial.println("forward"); 
      Forward(); 
      delay(currState.time); 
    }
    else if (s == right) {
      Serial.println("right"); 
      TurnRight(); 
      delay(currState.time); 
    }
    else if (s == left) {
      Serial.println("left"); 
      TurnLeft(); 
      delay(currState.time); 
    }
    else if (s == stop) {
      Serial.println("stop"); 
      Stop();
      delay(currState.time); 
    }
  }
}


/* Main code for line memorization and retracing 
 * Calling this function once will wait for impact switch to be pushed, 
 * then it will memorize a line and stop when the switch is pushed again, 
 * wait some delay, then retrace it */
void lineMemorizeMain() {
  State path[500]; 
  int numStates = 500; 

  // Wait for back right impact switch to be pressed to start 
  while (!collision_66.isCollision()) {
    Serial.println("Waiting to memorize"); 
  }
  memorizeLine(path, numStates); 
  
  // Wait for back left to start retracing 
  while (!collision_66.isCollision()) {
    Serial.println("Waiting to retrace"); 
  }
  retraceLine(path, numStates); 
  Serial.println("Done Retracing");

  Stop(); 
  _delay(60); // Just wait because don't want to run again 
}



/* ----------------------------------------------------------------------------------------------- */
/* MAIN LOOP 
/* ----------------------------------------------------------------------------------------------- */



/* 
 * Main Loop 
 * Each time the function runs it runs each course (so should only need to run once)
 */
void loop() {

  // Wait for back left to be pressed, then line follow
//  while (!collision_66.isCollision()) {
//    Serial.println("Waiting for line follow"); 
//  }
//  lineFollow(); 
  
   //Wait for back left to be pressed, then run line follow with obstacle avoidance
  while (!collision_66.isCollision()) {
    Serial.println("Waiting for obstacle avoidance"); 
  }
  lineFollow(); 

  // Then line memorize (function waits for back right to be pressed)
  //lineMemorizeMain(); 
}
