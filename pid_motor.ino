// Code from curiores' github
// This alternate version of the code does not require
// atomic.h. Instead, interrupts() and noInterrupts() 
// are used. Please use this code if your 
// platform does not support ATOMIC_BLOCK.

// A class to compute the control signal
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

// How many motors
#define NMOTORS 2

#define LEDC_FREQ 10000
#define LEDC_RES 8

// Pins
const int enca[2] = {33,35};
const int encb[2] = {32,34};
const int pwm[2] = {13,25}; //(PWMB, PWMA)
const int in1[2] = {12,27};
const int in2[2] = {14,26};
//const int ledcChans[2] = {0, 1}; // ledc channels
// Globals
long prevT = 0;
volatile int posi[] = {0,0};

// PID class instances
SimplePID pid[NMOTORS];

void setup() {
  Serial.begin(9600);
  ledcAttachPin(pwm[0], 0);
  ledcSetup(0, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(pwm[1], 1);
  ledcSetup(1, LEDC_FREQ, LEDC_RES);

  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pinMode(in1[k],OUTPUT);
    pinMode(in2[k],OUTPUT);
    pid[k].setParams(1,0,0,255);
  }
  
  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder<1>,RISING);
  
  Serial.println("target pos");
}

void loop() {

  // set target position
  int target[NMOTORS];
  target[0] =   250*sin(prevT/1e6);
  target[1] = -250*sin(prevT/1e6);

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position
  int pos[NMOTORS];
  noInterrupts(); // disable interrupts temporarily while reading
  for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  interrupts(); // turn interrupts back on
  
  // loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    // evaluate the control signal
    pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
    // signal the motor
    setMotor(dir,pwr,k,in1[k],in2[k]);
  }

  for(int k = 0; k < NMOTORS; k++){
    Serial.print(target[k]);
    Serial.print(" ");
    Serial.print(pos[k]);
    Serial.print(" ");
  }
  Serial.println();
}

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

template <int j>
void readEncoder(){
  int b = digitalRead(encb[j]);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
}