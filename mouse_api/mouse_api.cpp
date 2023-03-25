#include "mouse_api.h"

void MicroMouse::begin() {
  // setup sensors
  // start measurement thread
}

void MicroMouse::setMotorSpeeds(int pwm_l, int pwm_r) {
  //set speeds for motors. ints can be negative
  //remember to get values from config.h
}
void MicroMouse::goStraightForDistance(int distance) {
  //get current encoder position
  //call tickPIDcentering function
  //check if encoder distance has been reached
  //repeat
}
void MicroMouse::goStraightUntilNode() {
  //save current encoder position
  //call tickPIDcentering function
  //check sides closer than MAX_SIDE_DISTANCE
  //check front farther away than MAX_FRONT_DISTANCE
  //NOTE: use isWallsX functions for this
  //repeat
}
void MicroMouse::goStraightUntilWall() {
  //save current encoder position
  //call tickPIDcentering function
  //check front wall farther than MAX_FRONT_DISTANCE
  //repeat
}

bool MicroMouse::isWallsL() {
  //memsafely check if wall left closer than MAX_SIDE_DISTANCE
  //use mutexes etc
}
bool MicroMouse::isWallsF() {
  //memsafely check if front closer than MAX_FRONT_DISTANCE
}
bool MicroMouse::isWallsR() {
  //memsafely check if wall right closer than MAX_SIDE_DISTANCE
}
// get the current direction of the mouse. The initial direction is 0,
// with right turns adding 1 and left turns subtracting 1. The value
// is between 0 and 3
int MicroMouse::getDirection() {
  //return _facing
}
void MicroMouse::turnLeft() {
  //use PID to set targets of wheels to current encoder position +- CLICKS FOR ROT
  //update _facing
}
void MicroMouse::turnRight() {
  //use PID to set wheel targets to -+ CLICKS FOR ROT
  //update _facing
}
void MicroMouse::turnAround() {
  //use PID to set wheel targets to 2*CLICKS FOR ROT
  //update _facing
}
