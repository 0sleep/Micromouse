// physical wiring configuration
#define ENC_A_L 33
#define ENC_B_L 32
#define PWM_L 13
#define IN1_L 12
#define IN2_L 14

#define ENC_A_R 35
#define ENC_B_R 34
#define PWM_R 25
#define IN1_R 27
#define IN2_R 26

#define SHT_LOXL 15
#define SHT_LOXF 2
#define SHT_LOXR 4

// physical measurements
#define CLICKS_PER_ROT 60 //encoder counts per rotation of wheel
#define WHEEL_DISTANCE_AROUND 188 //mm per rotation
#define DISTANCE_TO_FRONT_WALL 100
#define DISTANCE_BETWEEN_SENSORS 100

// sensor addresses
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
// PWM configuration
#define LEDC_FREQ 10000
#define LEDC_RES 8
//wifi debugging configuration
#define WIFI_APN_NAME Micromouse
#define WIFI_APN_PASS Micromouse
#define WIFI_SOCKET_PORT 1234

#define DEBUG_ENABLED 1
