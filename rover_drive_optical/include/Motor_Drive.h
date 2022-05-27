#include <../lib/Robojax-L298N-DC-Motor/Robojax_L298N_DC_motor.h>
// motor defns
// motor 1 settings
#define CHA 0
#define ENA 14 // D10 // this pin must be PWM enabled pin if Arduino board is used
#define IN1 15 // D12
#define IN2 22 // D3
// motor 2 settings
#define IN3 17 // D8
#define IN4 16 // D9
#define ENB 4  // D11 this pin must be PWM enabled pin if Arduino board is used
#define CHB 1
const int CCW = 2; // do not change
const int CW = 1;  // do not change
#define motor1 1   // do not change
#define motor2 2   // do not change

// for two motors without debug information // Watch video instruciton for this line: https://youtu.be/2JTMqURJTwg
Robojax_L298N_DC_motor robot(IN1, IN2, ENA, CHA, IN3, IN4, ENB, CHB);