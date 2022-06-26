#include <pid_loops.h>

float timeCounterStraightLoop = 0;
float timeCounterRotateLoop = 0;

// distance PD loop
float R_pid_loop(float dist_error,
                 float prev_dist_error,
                 float integral_error) {
  float dist_derivative = dist_error - prev_dist_error;
  float kp_dist = 0.5;
  float ki_dist = 0;
  float kd_dist = 0.08;
  float R_pid = kp_dist * dist_error + kd_dist * dist_derivative +
                ki_dist * integral_error;
  R_pid = maxlimit(100, R_pid);
  return R_pid;
}

// angular correction PD loop
float theta_pid_loop(float theta_error, float prev_theta_error) {
  float theta_derivative = theta_error - prev_theta_error;
  float kp_theta = 15;  // change
  float kd_theta = 0;
  float theta_pid = kp_theta * theta_error + kd_theta * theta_derivative;
  return theta_pid;
}

// turn PD loop
float turn_pid_loop(float turn_error, float prev_turn_error) {
  float turn_derivative = turn_error - prev_turn_error;
  float kp_turn = 0.35;
  float kd_turn = 0.07;
  float turn_pid = kp_turn * turn_error + kd_turn * turn_derivative;
  return turn_pid;
}

// offset correction PD loop
float offset_pid_loop(float offset_error, float prev_offset_error) {
  float offset_derivative = offset_error - prev_offset_error;
  float kp_offset = 0.0;
  float kd_offset = 0.0;
  float offset_pid = kp_offset * offset_error + kd_offset * offset_derivative;
  return offset_pid;
}

// motor control straight
// takes distance in mm
void rover_straight(float dist_reqd) {
    float current_dist_error = dist_reqd;
    float current_integral_dist_error = 0;
    float current_turn_error = 0;
    // reset_imu_angle();
    while (abs(current_dist_error) > max_dist_error) {
        //float deltat = 0;
        check_cumulative_dist();
        /*check_imu_angle(delta_theta_left, delta_theta_right, total_theta_left,
                        total_theta_right, deltat);*/
        //timeCounterStraightLoop += deltat;

        float prev_dist_error = current_dist_error;
        current_dist_error =
                current_dist_error - (delta_v_mm_left + delta_v_mm_right) / 2;
        current_integral_dist_error =
                current_integral_dist_error + current_dist_error;

        float prev_turn_error = current_turn_error;
        current_turn_error = delta_v_mm_right - delta_v_mm_left;

        if ((abs(current_turn_error) < max_turn_error)|| (abs(current_dist_error)<10)){
          current_turn_error=0;
        }

        float R_pid = R_pid_loop(current_dist_error, prev_dist_error,
                                 current_integral_dist_error);
        float turn_pid = turn_pid_loop(current_turn_error, prev_turn_error);

        float leftmotorcontrol = maxlimit(100, R_pid + turn_pid);
        float rightmotorcontrol = maxlimit(100, R_pid - turn_pid);

        Serial.print("Left motor control: " + String(leftmotorcontrol));
        Serial.println(", Right motor control: " + String(rightmotorcontrol));
        Serial.println("Turn pid: "+String(turn_pid));

        motorrotate(leftmotorcontrol, motor1);
        motorrotate(rightmotorcontrol, motor2);

        delay(5);
    }
    robot.brake(motor1);
    robot.brake(motor2);
    last_speed = 0;
}

float modulo_2pi(float input) {
    int divd = floor(input / (2 * PI));
    float rem = input - float(divd * 2 * PI);
    return rem;
}

float angle_2pi(float input) {
    return modulo_2pi(abs(input));
}

float angle_difference_2pi(float prev, float curr) {
    float diff1 = abs(prev - curr);
    float diff2 = abs(prev - curr + 2 * PI);
    float diff3 = abs(prev - curr - 2 * PI);
    if (diff1 < diff2 && diff1 < diff3) {
        return prev - curr;
    } else if (diff2 < diff1 && diff2 < diff3) {
        return prev - curr + 2 * PI;
    } else {
        return prev - curr - 2 * PI;
    }
}

void rover_rotate(float theta_reqd) {
    float current_theta_error = theta_reqd;
    float current_offset_error = 0;
    // reset_imu_angle();
    while (abs(current_theta_error) > max_theta_error) {
        check_cumulative_dist();

        float prev_theta_error = current_theta_error;
        current_theta_error =
                current_theta_error - (delta_theta_left + delta_theta_right) / 2;

        float prev_offset_error = current_offset_error;
        current_offset_error = delta_v_mm_right + delta_v_mm_left;
        
        if ((abs(current_offset_error) < max_offset_error)|| (abs(current_theta_error)<0.4)){
          current_offset_error = 0;
        }

        float theta_pid = theta_pid_loop(current_theta_error, prev_theta_error);
        float offset_pid = offset_pid_loop(current_offset_error, prev_offset_error);
        float leftmotorcontrol = maxlimit(100, theta_pid + offset_pid);
        float rightmotorcontrol = maxlimit(100, -theta_pid + offset_pid);

        motorrotate(leftmotorcontrol, motor1);
        motorrotate(rightmotorcontrol, motor2);

        Serial.println("Rotate loop: current_theta_error - " + String(abs(current_theta_error)) + "; max_theta_error: "
                       + String(max_theta_error) + " delta_theta_left: " + String(delta_theta_left) +
                       "; delta_theta_right: " + String(delta_theta_right));

        delay(15);
    }
    robot.brake(motor1);
    robot.brake(motor2);
    Serial.println("exiting the loop");
    last_speed = 0;
}