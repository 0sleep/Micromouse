//#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Adafruit_VL53L0X.h"
#include "freertos/semphr.h"
//#include "stdatomic.h"
// Task speeds
#define MOTOR_UPDATE_SPEED 100 //ms
#define SENSOR_UPDATE_SPEED 100 //ms
#define BRAIN_UPDATE_SPEED 100 //ms
#define CLICKS_PER_ROT 60 //encoder counts per rotation of wheel
#define WHEEL_DISTANCE_AROUND 188 //mm per rotation
#define DISTANCE_TO_FRONT_WALL 100
// sensor addresses
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
// set the pins to shutdown
#define SHT_LOX1 15
#define SHT_LOX2 2
#define SHT_LOX3 4

#define LEDC_FREQ 10000
#define LEDC_RES 8

class SimplePID{
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage
  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}
  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }
  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    // error
    int e = target - value;
    // derivative
    float dedt = (e-eprev)/(deltaT);
    // integral
    eintegral = eintegral + e*deltaT;
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    }
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
    // store previous error
    eprev = e;
  }
};

//set motor speeds
void setMotor(int dir, int pwmVal, int chan, int in1, int in2){
  ledcWrite(chan,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

int distanceToCounts(int distanceMM) { // converts distance from sensor into encoder counts. depends on counts per rotation and wheel size
  return (distanceMM*CLICKS_PER_ROT)/WHEEL_DISTANCE_AROUND;
}
int distance[3]; //holds sensor distance
int target[2]; //holds motor targets

SemaphoreHandle_t xSensorInitDone = NULL;//notify brain task that sensor setup is done. replace with notification ideally
SemaphoreHandle_t xDistanceAvailable = NULL; //protect distance[3]
SemaphoreHandle_t xTargetReadable = NULL; //protect target[2]
static portMUX_TYPE xLeftEncoder; //ISR spinlocks, https://esp32.com/viewtopic.php?t=12621
static portMUX_TYPE xRightEncoder;
volatile int posi[2] = {0,0}; // actual current position, modified by interrupt
// Pins
const int enca[2] = {33,35};
const int encb[2] = {32,34};
const int pwm[2] = {13,25}; //(PWMB, PWMA)
const int in1[2] = {12,27};
const int in2[2] = {14,26};
//Sensor setup stuff
//VL53L0X_RangingMeasurementData_t measure_temp[3]; // array to hold temporary ranges before semaphore-protected copying into distances
//int dist_temp[3];//array to hold actual distances before copy, is "volatile"
// tasks
void TaskReadSensor( void *pvParameters); //continuously. block with semaphore. NOTE: no longer a task due to I2C getting angry at Tasks. Now in loop()
void TaskCommandMotors( void *pvParameters); //do PID calc
void TaskBrainControl(void *pvParameters); //use TaskReadSensor data to pass commandings to TaskCommandMotors


void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  printf("Init Start\n");
  xSensorInitDone = xSemaphoreCreateMutex();
  xDistanceAvailable = xSemaphoreCreateMutex(); // or xSemaphoreCreateBinary(); 
  xTargetReadable = xSemaphoreCreateMutex();
  spinlock_initialize(&xLeftEncoder);
  spinlock_initialize(&xRightEncoder);
  // disableCore0WDT(); // disable pesky watchdog. for some reason sensor init takes forever, which means watchdog kills it before it's done
  // Now set up tasks to run independently.
  printf("Create Tasks\n");
  xTaskCreate(
    TaskReadSensor
    ,  "Sensor Read"   // A name just for humans
    ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater, read https://esp32.com/viewtopic.php?t=11514 for implementation help
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest. (Note: MAX_PRIORITIES = 25)
    ,  NULL );
  xTaskCreate(
    TaskCommandMotors
    ,  "Motor PID"
    ,  2048  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );
  xTaskCreate(
    TaskBrainControl
    ,  "Brain Function"
    , 2048
    , NULL
    , 2
    , NULL
  );
  printf("Tasks setup done\n");
}

void loop() {
  vTaskDelay(100/portTICK_PERIOD_MS); // empty loop
}

void TaskReadSensor(void *pvParameters) {
  xSemaphoreTake(xSensorInitDone, portMAX_DELAY);
  printf("Sensor init start\n");
  (void) pvParameters;
  VL53L0X_RangingMeasurementData_t measure_temp[3]; // array to hold temporary ranges before semaphore-protected copying into distances
  int dist_temp[3];//array to hold actual distances before copy
  // objects for the vl53l0x
  Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
  Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
  Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
  // do sensor init stuff
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  vTaskDelay(10/portTICK_PERIOD_MS);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  vTaskDelay(10/portTICK_PERIOD_MS);
  // activating LOX1 and resetting LOX2 ---------------------------------------------------
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  printf("Sensor init LOX1\n");
  // initing LOX1n                              
  if(!lox1.begin(LOX1_ADDRESS)) {
    printf("Failed to boot first VL53L0X\n");
    while(1);
  }
  printf("Sensor finish LOX1\n");
  vTaskDelay(10/portTICK_PERIOD_MS);
  // activating LOX2 ----------------------------------------------------------------------
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, LOW);
  vTaskDelay(10/portTICK_PERIOD_MS);
  //initing LOX2
  printf("Sensor init LOX2\n");
  if(!lox2.begin(LOX2_ADDRESS)) {
    printf("Failed to boot second VL53L0X\n");
    //while(1);
  }
  printf("Sensor finish LOX2\n");
  // activating LOX3 ----------------------------------------------------------------------
  digitalWrite(SHT_LOX3, HIGH);
  vTaskDelay(10/portTICK_PERIOD_MS);
  //initing LOX2
  printf("Sensor init LOX3\n");
  if(!lox3.begin(LOX3_ADDRESS)) {
    printf("Failed to boot third VL53L0X\n");
    //while(1);
  }
  printf("Sensor finish LOX3\n");
  printf("Sensor init done\n");
  xSemaphoreGive(xSensorInitDone);
  for (;;) {
    //infinite reading loop
    lox1.rangingTest(&measure_temp[0], false);
    lox2.rangingTest(&measure_temp[1], false);
    lox3.rangingTest(&measure_temp[2], false);
    if(measure_temp[0].RangeStatus != 4) {     // if not out of range
      dist_temp[0] = measure_temp[0].RangeMilliMeter;
    } else {
      dist_temp[0] = -1;
    }
    if(measure_temp[1].RangeStatus != 4) {
      dist_temp[1] = measure_temp[1].RangeMilliMeter;
    } else {
      dist_temp[1] = -1;
    }
    if(measure_temp[2].RangeStatus != 4) {
      dist_temp[2] = measure_temp[2].RangeMilliMeter;
    } else {
      dist_temp[2] = -1;
    }
    printf("Sensors: %d %d %d\n", dist_temp[0], dist_temp[1], dist_temp[2]);
    xSemaphoreTake(xDistanceAvailable, portMAX_DELAY);
    memcpy(distance, dist_temp, sizeof(dist_temp[0])*3);
    xSemaphoreGive(xDistanceAvailable);
    vTaskDelay(SENSOR_UPDATE_SPEED/portTICK_PERIOD_MS);
  }
}


void TaskCommandMotors(void *pvParameters) {
  printf("Init motors start\n");
  // Globals
  long prevT = 0;
  int target_internal[2];
  // PID class instances
  SimplePID pid[2];
  //set up PWM
  ledcSetup(0, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(pwm[0], 0);
  ledcSetup(1, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(pwm[1], 1);
  for(int k = 0; k < 2; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pinMode(in1[k],OUTPUT);
    pinMode(in2[k],OUTPUT);
    pid[k].setParams(1,4,0,255);
  }
  //ATTACH INTERRUPTS SOMEWHERE
  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoderR, RISING);
  printf("Init motors finish\n");
  for (;;) {
    //set target position
    xSemaphoreTake(xDistanceAvailable, portMAX_DELAY);
    target_internal[0] = -target[0];
    target_internal[1] = target[1];
    xSemaphoreGive(xDistanceAvailable);
    //time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;
    //read the position
    int pos[2];
    portENTER_CRITICAL(&xLeftEncoder);
    pos[0] = posi[0];//safely read left encoder
    portEXIT_CRITICAL(&xLeftEncoder);
    portENTER_CRITICAL(&xRightEncoder);
    pos[1] = posi[1];//safely read right encoder
    portEXIT_CRITICAL(&xRightEncoder);
    //loop through motors
    for(int k = 0; k < 2; k++){
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target_internal[k],deltaT,pwr,dir);
      // signal the motor
      setMotor(dir,pwr,k,in1[k],in2[k]);
    }
    vTaskDelay(MOTOR_UPDATE_SPEED/portTICK_PERIOD_MS);
  }
}

// I have decided not to use template functions duet to complexity. Spinlocks also disable interrupts, yay
void readEncoderL() {
  int b = digitalRead(encb[0]);
  if(b>0){
    posi[0]++;
  } else{
    posi[0]--;
  }
}
void readEncoderR() {
  int b = digitalRead(encb[1]);
  if(b>0){
    posi[1]++;
  } else{
    posi[1]--;
  }
}

void TaskBrainControl(void *pvParameters) {
  printf("Init Brain start\n");
  int localTarget[2];
  int localMeasures[3];
  vTaskDelay(2000/portTICK_PERIOD_MS);
  xSemaphoreTake(xSensorInitDone, portMAX_DELAY);
  printf("Init Brain done\n");
  for (;;) {
    //read the position
    int pos[2]; // hold current position
    portENTER_CRITICAL(&xLeftEncoder);
    pos[0] = -posi[0];//safely read left encoder
    portEXIT_CRITICAL(&xLeftEncoder);
    portENTER_CRITICAL(&xRightEncoder);
    pos[1] = posi[1];//safely read right encoder
    portEXIT_CRITICAL(&xRightEncoder);
    //set motor targets to front sensor distance - 100mm
    // get sensor values
    xSemaphoreTake(xDistanceAvailable, portMAX_DELAY);
    memcpy(localMeasures, distance, sizeof(distance[0])*3);
    xSemaphoreGive(xDistanceAvailable);
    //calculated targets
    localTarget[0] = pos[0]+distanceToCounts(localMeasures[1]-DISTANCE_TO_FRONT_WALL);
    localTarget[1] = pos[1]+distanceToCounts(localMeasures[1]-DISTANCE_TO_FRONT_WALL);
    printf("Current %d %d\n Add %d \n Targets %d %d \n", pos[0], pos[1], distanceToCounts(localMeasures[1]-30), localTarget[0], localTarget[1]);

    //set targets
    xSemaphoreTake(xTargetReadable, portMAX_DELAY);
    //memcpy(target, localTarget, sizeof(localTarget[0]*2));
    target[0] = localTarget[0];
    target[1] = localTarget[1];
    //printf("Target %d %d \n Internal %d %d \n", target[0], target[1], localTarget[0], localTarget[1]);
    xSemaphoreGive(xTargetReadable);
    // output things
    //printf("Distance: %d Target: %d\n", localMeasures[1], localTarget[0]);
    // wait for a bit
    vTaskDelay(BRAIN_UPDATE_SPEED/portTICK_PERIOD_MS);
  }
}
/*
TODO
----
- Combine all startups / semaphore shit etc. might fix slow sensor lib
- Test and compile, gl|hf
*/