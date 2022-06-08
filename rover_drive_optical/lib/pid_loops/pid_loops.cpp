#include <pid_loops.h>

// distance PD loop
float R_pid_loop(float dist_error,
                 float prev_dist_error,
                 float integral_error) {
  float dist_derivative = dist_error - prev_dist_error;
  float kp_dist = 10;
  float ki_dist = 0;
  float kd_dist = 0;
  float R_pid = kp_dist * dist_error + kd_dist * dist_derivative +
                ki_dist * integral_error;
  R_pid = maxlimit(100, R_pid);
  return R_pid;
}

// angular correction PD loop
float theta_pid_loop(float theta_error, float prev_theta_error) {
  float theta_derivative = theta_error - prev_theta_error;
  float kp_theta = 25;  // change
  float kd_theta = 0;
  float theta_pid = kp_theta * theta_error + kd_theta * theta_derivative;
  return theta_pid;
}

// turn PD loop
float turn_pid_loop(float turn_error, float prev_turn_error) {
  float turn_derivative = turn_error - prev_turn_error;
  float kp_turn = 0;
  float kd_turn = 0;
  float turn_pid = kp_turn * turn_error + kd_turn * turn_derivative;
  return turn_pid;
}

// offset correction PD loop
float offset_pid_loop(float offset_error, float prev_offset_error) {
  float offset_derivative = offset_error - prev_offset_error;
  float kp_offset = 0.;
  float kd_offset = 0;
  float offset_pid = kp_offset * offset_error + kd_offset * offset_derivative;
  return offset_pid;
}

// motor control straight
// takes distance in mm
void rover_straight(float dist_reqd) {
  float current_dist_error = dist_reqd;
  float current_integral_dist_error = 0;
  float current_turn_error = 0;
  while (abs(current_dist_error) > max_dist_error) {
    check_cumulative_dist();
    float prev_dist_error = current_dist_error;
    current_dist_error =
        current_dist_error - (delta_v_mm_left + delta_v_mm_right) / 2;
    current_integral_dist_error =
        current_integral_dist_error + current_dist_error;

    float prev_turn_error = current_turn_error;
    current_turn_error = delta_v_mm_right - delta_v_mm_left;
    // condition to exit loop
    if (abs(current_dist_error) < max_dist_error) {
      return;
    }

    float R_pid = R_pid_loop(current_dist_error, prev_dist_error,
                             current_integral_dist_error);
    float turn_pid = turn_pid_loop(current_turn_error, prev_turn_error);

    float leftmotorcontrol = maxlimit(100, R_pid + turn_pid);
    float rightmotorcontrol = maxlimit(100, R_pid - turn_pid);

    motorrotate(leftmotorcontrol, motor1);
    motorrotate(rightmotorcontrol, motor2);
    /*
    if ((millis() - last_print) > 1000) {
      Serial.println("Current dist error " + String(current_dist_error) +
                     "Prev dist error " + String(prev_dist_error));
      Serial.println("Left motor control " + String(leftmotorcontrol) +
                     ", Right motor control " + String(rightmotorcontrol));
      Serial.println("\n");
      last_print = millis();
    }*/
  }
  robot.brake(motor1);
  robot.brake(motor2);
}

float modulo_2pi(float input) {
  int divd = floor(input / (2 * PI));
  float rem = input - float(divd * 2 * PI);
  return rem;
}

void rover_rotate(float theta_reqd) {
  float current_theta_error = theta_reqd;
  float current_offset_error = 0;
  while (abs(current_theta_error) > max_theta_error) {
    check_cumulative_dist();
    float prev_theta_error = current_theta_error;
    current_theta_error =
        current_theta_error - (delta_theta_left + delta_theta_right) / 2;

    float prev_offset_error = current_offset_error;
    current_offset_error = delta_v_mm_right + delta_v_mm_left;
    // condition to exit loop
    if (abs(current_theta_error) < max_theta_error) {
      return;
    }
    float theta_pid = theta_pid_loop(current_theta_error, prev_theta_error);
    float offset_pid = offset_pid_loop(current_offset_error, prev_offset_error);
    float leftmotorcontrol = maxlimit(100, theta_pid - offset_pid);
    float rightmotorcontrol = maxlimit(100, -theta_pid - offset_pid);

    motorrotate(leftmotorcontrol, motor1);
    motorrotate(rightmotorcontrol, motor2);

    Serial.println(String(modulo_2pi(total_theta_left)) + "," +
                   String(modulo_2pi(total_theta_right)) + "," +
                   String(current_theta_error));

    /*if ((millis() - last_print) > 1000) {
      Serial.println("Total_theta: (" + String(total_theta_left) + "," +
                     String(total_theta_right) + ")|Average: " +
                     String((total_theta_left + total_theta_right) / 2));
      Serial.println("Current theta error " + String(current_theta_error) +
                     "Prev theta error " + String(prev_theta_error));
      Serial.println("Left motor control " + String(leftmotorcontrol) +
                     ", Right motor control " + String(rightmotorcontrol));
      Serial.println("\n");
      last_print = millis();
    }*/

    delay(50);
  }
  robot.brake(motor1);
  robot.brake(motor2);
}

/*
// main motor control function
void motor_control(float dist_reqd, float theta_reqd) {
  float current_dist_error = dist_reqd;
  float current_theta_error = theta_reqd;
  // while ((abs(current_dist_error) > max_dist_error) ||
  // (abs(current_theta_error) > max_theta_error))
  bool enable_pid_bool = enable_pid(dist_reqd, theta_reqd, current_dist_error,
                                    current_theta_error);
  while (enable_pid_bool) {
    check_cumulative_dist();
    float prev_dist_error = current_dist_error;
    current_dist_error = current_dist_error - delta_r;

    float prev_theta_error = current_theta_error;
    current_theta_error = current_theta_error - delta_theta;

    // returns if pid no longer required
    enable_pid_bool = enable_pid(dist_reqd, theta_reqd, current_dist_error,
                                 current_theta_error);
    if (!enable_pid_bool) {
      return;
    }

    float R_pid = R_pid_loop(current_dist_error, prev_dist_error);
    float theta_pid = theta_pid_loop(current_theta_error, prev_theta_error);
    // Serial.print("R_pid " + String(R_pid));
    // Serial.println(", theta_pid " + String(theta_pid));

    float leftmotorcontrol = maxlimit(100, R_pid + theta_pid);
    float rightmotorcontrol = maxlimit(100, R_pid - theta_pid);
    // Serial.println("Left motor control "+String(leftmotorcontrol)+", Right
    // motor control "+String(rightmotorcontrol));
    motorrotate(leftmotorcontrol, motor1);
    motorrotate(rightmotorcontrol, motor2);

    if ((millis() - last_print) > 1000) {
      Serial.println("Total_l,Total_r: (" + String(total_l) + "," +
                     String(total_r) + ")");
      Serial.println("Total_Thetha : " + String(total_theta));
      Serial.println("Current dist error " + String(current_dist_error) +
                     "Prev dist error " + String(prev_dist_error));
      Serial.println("Current theta error " + String(current_theta_error) +
                     "Prev theta error " + String(prev_theta_error));
      Serial.println("Left motor control " + String(leftmotorcontrol) +
                     ", Right motor control " + String(rightmotorcontrol));
      Serial.println("\n");
      last_print = millis();
    }
    delay(100);
  }
}*/