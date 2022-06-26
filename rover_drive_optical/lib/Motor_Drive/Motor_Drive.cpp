#include <Motor_Drive.h>

// for two motors without debug information // Watch video instruciton for this
// line: https://youtu.be/2JTMqURJTwg
Robojax_L298N_DC_motor robot(IN1, IN2, ENA, CHA, IN3, IN4, ENB, CHB);

int last_speed = 0;
const int step_size_motor = 30;

// limiting function
float maxlimit(float max, float input) {
  float out;
  if (abs(input) > max) {
    if (input > 0) {
      out = max;
    } else {
      out = -max;
    }
  } else {
    out = input;
  }
  return out;
}

#if 1
// motor profiling function ie sets limits for the minimum motor power
int motor_profile(int preadj_speed) {
  float adj_speed = 0;
  if (preadj_speed > 0) {
    adj_speed = (preadj_speed / 100.0) * (max_motor_val - min_motor_val) +
                min_motor_val;
  } else if (preadj_speed < 0) {
    adj_speed = (preadj_speed / 100.0) * (max_motor_val - min_motor_val) -
                min_motor_val;
  }
  return (int)adj_speed;
  /*return (x == 0) ? 0
         : (x > 0)
             ? (x / 100) * (max_motor_val - min_motor_val) + min_motor_val
             : (x / 100) * (max_motor_val - min_motor_val) - min_motor_val;*/
}
#else
int motor_profile(float preadj_speed) {
  int adj_speed_lookup[101] = {
      0,  0,  0,  1,  1,  1,  1,  2,  2,  2,  3,  3,  4,  4,  5,   5,  6,
      6,  7,  8,  8,  9,  10, 11, 12, 12, 13, 14, 15, 16, 17, 18,  19, 20,
      21, 22, 24, 25, 26, 27, 28, 30, 31, 32, 34, 35, 36, 38, 39,  41, 42,
      44, 45, 47, 48, 50, 51, 53, 54, 56, 57, 59, 60, 62, 63, 65,  66, 68,
      69, 71, 72, 74, 75, 77, 78, 79, 81, 82, 83, 84, 86, 87, 88,  89, 90,
      91, 92, 93, 94, 95, 95, 96, 97, 97, 98, 98, 99, 99, 99, 100, 100};
  if (preadj_speed > 0) {
    return adj_speed_lookup[int(preadj_speed)];
  } else {
    return -adj_speed_lookup[int(preadj_speed)];
  }
}

#endif

// motor function (to remove need for CCW and CW -> -100 to 100)
void motorrotate(int speed, int motor_no) {
  /*
  if (abs(speed - last_speed) > step_size_motor) {
    if (speed > last_speed){
      speed = last_speed + step_size_motor;
    }
    else {
      speed = last_speed - step_size_motor;
    }
  }
  */
  if (speed > 0) {
    robot.rotate(motor_no, motor_profile(speed), CCW);
  } else {
    robot.rotate(motor_no, motor_profile(abs(speed)), CW);
  }
  last_speed = speed;
}