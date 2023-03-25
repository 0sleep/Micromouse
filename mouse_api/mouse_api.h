#ifndef MOUSE_API_H
#define MOUSE_API_H


#include "config.h"
#include "Adafruit_VL53L0X.h"

class MicroMouse {
  private:
    // current encoder positions
    volatile int _posi_l = 0;
    volatile int _posi_r = 0;
    // current encoder positions, safe
    int _pos_l;
    int _pos_r;
    // current facing direction
    int _facing;
    // sensor objects for the vl53l0x ToF sensors
    Adafruit_VL53L0X _lox_l;
    Adafruit_VL53L0X _lox_f;
    Adafruit_VL53L0X _lox_r;
    // sensor reading objects
    VL53L0X_RangingMeasurementData_t _measure_l;
    VL53L0X_RangingMeasurementData_t _measure_f;
    VL53L0X_RangingMeasurementData_t _measure_r;
    // wrapper for motor speed setting
    void _setMotorSpeeds(int pwm_l, int pwm_r);
    // interrupt function for encoders to set motor positions
    void _readEncoderL();
    void _readEncoderR();
    //function to handle PID calculations
    void _tickPIDcentering();
    // function to move mouse in a straight line by x clicks based on encoder clicks
    void _forward(int distance);
  public:
    // set-up sensors and mouse hardware
    // start measurement loop
    void begin();
    // go in a straight line for a specified distance
    // use PID to avoid side collisions
    void goStraightForDistance(int distance);
    // same as previous, but stop once a node/juction/dead end has been reached
    void goStraightUntilNode();
    // same as previous, but stop only when a wall has been reached
    void goStraightUntilWall();
    // check if a wall is in range on the left
    bool isWallsL();
    // check if a wall is in range at the front
    bool isWallsF();
    // check if a wall is in range on the right
    bool isWallsR();
    // get the current direction of the mouse. The initial direction is 0,
    // with right turns adding 1 and left turns subtracting 1. The value
    // is between 0 and 3
    int getDirection();
    // rotate the mouse on the spot by 90° left
    void turnLeft();
    // rotate the mouse on the spot by 90° right
    void turnRight();
    // spin the mouse 180°
    void turnAround();
};

#endif
