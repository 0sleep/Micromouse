#include "mouse_api.h"

void MicroMouse::begin() {
  printf("Init Start\n");
  // setup sensors
  // do sensor init stuff
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  // activating LOX1 and resetting LOX2 ---------------------------------------------------
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  printf("Sensor init LOX1\n");
  // initing LOX1n
  if (!_lox_l.begin(LOX1_ADDRESS))
  {
    printf("Failed to boot first VL53L0X\n");
    while (1)
      ;
  }
  printf("Sensor finish LOX1\n");
  vTaskDelay(10 / portTICK_PERIOD_MS);
  // activating LOX2 ----------------------------------------------------------------------
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, LOW);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  // initing LOX2
  printf("Sensor init LOX2\n");
  if (!_lox_f.begin(LOX2_ADDRESS))
  {
    printf("Failed to boot second VL53L0X\n");
    // while(1);
  }
  printf("Sensor finish LOX2\n");
  // activating LOX3 ----------------------------------------------------------------------
  digitalWrite(SHT_LOX3, HIGH);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  // initing LOX2
  printf("Sensor init LOX3\n");
  if (!_lox_r.begin(LOX3_ADDRESS))
  {
    printf("Failed to boot third VL53L0X\n");
    // while(1);
  }
  _lox_l.startRangingContinuous();
  _lox_f.startRangingContinuous();
  _lox_r.startRangingContinuous();
  printf("Sensor finish LOX3\n");
  printf("Sensor init done\n");
  // start measurement thread
  xTaskCreate(
    this->TaskSensor, 
    "SENSOR TASK",
    2048,
    NULL,
    1,
    NULL
  );
}

void MicroMouse::_TaskReadSensor(void *pvParameters) {
  if (_lox_l.isRangeComplete()){
    //sem
    _lox_l.readRange();
  }
   //read sensor
   //get semaphore
   //write
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
