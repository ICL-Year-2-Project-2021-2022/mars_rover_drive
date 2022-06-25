#include <Robojax_L298N_DC_motor.h>
// motor defns
// motor 1 settings
#define CHA 0
#define ENA 14 // D10 PWMA // this pin must be PWM enabled pin if Arduino board is used
#define IN1 15 // D12 A11
#define IN2 2 // D13 A12
// motor 2 settings
#define IN3 21 // D3 B11
#define IN4 22 // D4 B12
#define ENB 4  // D11 PWMB this pin must be PWM enabled pin if Arduino board is used
#define CHB 1
const int CCW = 2; // do not change
const int CW = 1;  // do not change
#define motor1 1   // do not change
#define motor2 2   // do not change

const int min_motor_val = 20;
const int max_motor_val = 60;

// for two motors without debug information // Watch video instruciton for this line: https://youtu.be/2JTMqURJTwg
extern Robojax_L298N_DC_motor robot;

float maxlimit(float max, float input);

int motor_profile(float preadj_speed);

void motorrotate(int speed, int motor_no);