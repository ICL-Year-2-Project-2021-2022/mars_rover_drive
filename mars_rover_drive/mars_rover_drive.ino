#include <Robojax_L298N_DC_motor.h>
// motor 1 settings
#define CHA 0
#define ENA 19 // this pin must be PWM enabled pin if Arduino board is used
#define IN1 18
#define IN2 5
// motor 2 settings
#define IN3 17
#define IN4 16
#define ENB 4// this pin must be PWM enabled pin if Arduino board is used
#define CHB 1
const int CCW = 2; // do not change
const int CW  = 1; // do not change
#define motor1 1 // do not change
#define motor2 2 // do not change
// for two motors without debug information // Watch video instruciton for this line: https://youtu.be/2JTMqURJTwg
Robojax_L298N_DC_motor robot(IN1, IN2, ENA, CHA,  IN3, IN4, ENB, CHB);
// for two motors with debug information
//Robojax_L298N_DC_motor robot(IN1, IN2, ENA, CHA, IN3, IN4, ENB, CHB, true);
void setup() {
  Serial.begin(115200);
  robot.begin();
  //L298N DC Motor by Robojax.com
}
void loop() {

  Serial.println("xxx-----xxx-----xxx-----xxx");
  Serial.println("Enter X: ");
  while (Serial.available()==0){}             // wait for user input
  int x = Serial.parseInt();

  Serial.println("Enter Y: ");
  while (Serial.available()==0){}             // wait for user input
  int y = Serial.parseInt();

  
  
}

int cartesian_to_angle(x,y){
  int base_angle atan(abs(y)/abs(x));
  if (x >= 0 && y >= 0){
    return base_angle;
  }
  else if (x < 0 && y >= 0){
    return (180 - base_angle);
  }
  else if (x < 0 && y < 0){
    return (-180 + base_angle);
  }
  else {
    return (-base_angle);
  }
}

int cartesian_to_dist(x,y){
  return sqrt(y^2 + x^2);
}
