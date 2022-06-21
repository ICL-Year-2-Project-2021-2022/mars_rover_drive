#include <Arduino.h>
#include <Motor_Drive.h>
#include <Optical_Flow.h>
//#include <IMU.h>

// max errors
const float max_dist_error = 0.5;
const float max_theta_error = 0.05;

// distance PD loop
float R_pid_loop(float dist_error, float prev_dist_error, float integral_error);
// angular correction PD loop
float theta_pid_loop(float theta_error, float prev_theta_error);

// turn PD loop
float turn_pid_loop(float turn_error, float prev_turn_error);

// offset correction PD loop
float offset_pid_loop(float offset_error, float prev_offset_error);

// motor control straight in mm
void rover_straight(float dist_reqd);

float modulo_2pi(float input);

void rover_rotate(float theta_reqd);
