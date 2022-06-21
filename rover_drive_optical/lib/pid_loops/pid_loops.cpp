#include <pid_loops.h>

float timeCounterStraightLoop = 0;
float timeCounterRotateLoop = 0;

// distance PD loop
float R_pid_loop(float dist_error,
                 float prev_dist_error,
                 float integral_error) {
    float dist_derivative = dist_error - prev_dist_error;
    float kp_dist = 2.5;
    float ki_dist = 0;
    float kd_dist = 2;
    float R_pid = kp_dist * dist_error + kd_dist * dist_derivative +
                  ki_dist * integral_error;
    R_pid = maxlimit(100, R_pid);
    return R_pid;
}

// angular correction PD loop
float theta_pid_loop(float theta_error, float prev_theta_error) {
    float theta_derivative = theta_error - prev_theta_error;
    float kp_theta = 20;  // change
    float kd_theta = 15;
    float theta_pid = kp_theta * theta_error + kd_theta * theta_derivative;
    return theta_pid;
}

// turn PD loop
float turn_pid_loop(float turn_error, float prev_turn_error) {
    float turn_derivative = turn_error - prev_turn_error;
    float kp_turn = 5;
    float kd_turn = 3;
    float turn_pid = kp_turn * turn_error + kd_turn * turn_derivative;
    return turn_pid;
}

// offset correction PD loop
float offset_pid_loop(float offset_error, float prev_offset_error) {
    float offset_derivative = offset_error - prev_offset_error;
    float kp_offset = 0.5;
    float kd_offset = 0.25;
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
        float deltat = 0;
        check_cumulative_dist();
        /*check_imu_angle(delta_theta_left, delta_theta_right, total_theta_left,
                        total_theta_right, deltat);*/
        timeCounterStraightLoop += deltat;

        float prev_dist_error = current_dist_error;
        current_dist_error =
                current_dist_error - (delta_v_mm_left + delta_v_mm_right) / 2;
        current_integral_dist_error =
                current_integral_dist_error + current_dist_error;

        float prev_turn_error = current_turn_error;
        current_turn_error = delta_v_mm_right - delta_v_mm_left;
        // condition to exit loop
        if (abs(current_dist_error) < max_dist_error) {
            break;
        }

        float R_pid = R_pid_loop(current_dist_error, prev_dist_error,
                                 current_integral_dist_error);
        float turn_pid = turn_pid_loop(current_turn_error, prev_turn_error);

        float leftmotorcontrol = maxlimit(100, R_pid - turn_pid);
        float rightmotorcontrol = maxlimit(100, R_pid + turn_pid);


        Serial.print("Left motor control: " + String(leftmotorcontrol));
        Serial.println(", Right motor control: " + String(rightmotorcontrol));
        Serial.println("Turn pid: "+String(turn_pid));

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
        //Serial.println("abs(current_dist_error)" + String(abs(current_dist_error)) + "" + String(max_dist_error));
        delay(10);
    }
    robot.brake(motor1);
    robot.brake(motor2);
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
        /*float deltat = 0;
        check_imu_angle(delta_theta_left, delta_theta_right, total_theta_left,
                        total_theta_right, deltat);
        timeCounterRotateLoop += deltat;
*/
        /*if (timeCounterStraightLoop <= 1.57f) {
            delta_theta_left = delta_theta_left + 0.0007f;
            delta_theta_right = delta_theta_right + 0.0007f;
        } else if (timeCounterStraightLoop <= 2.5f) {
            delta_theta_left = delta_theta_left + 0.0019f;
            delta_theta_right = delta_theta_right + 0.0019f;
        } else if (timeCounterStraightLoop <= 3.5f) {
            delta_theta_left = delta_theta_left + 0.0037f;
            delta_theta_right = delta_theta_right + 0.0037f;
        } else if (timeCounterStraightLoop <= 4.5f) {
            delta_theta_left = delta_theta_left + 0.0028f;
            delta_theta_right = delta_theta_right + 0.0028f;
        } else if (timeCounterStraightLoop <= 10) {
            delta_theta_left = delta_theta_left + 0.0001f;
            delta_theta_right = delta_theta_right + 0.0001f;
        } else {
            delta_theta_left = delta_theta_left + 9E-05f;
            delta_theta_right = delta_theta_right + 9E-05f;
        }*/

        float prev_theta_error = current_theta_error;
        current_theta_error =
                current_theta_error - (delta_theta_left + delta_theta_right) / 2;

        float prev_offset_error = current_offset_error;
        current_offset_error = delta_v_mm_right + delta_v_mm_left;
        // condition to exit loop
        if (abs(current_theta_error) < max_theta_error) {
            break;
        }
        float theta_pid = theta_pid_loop(current_theta_error, prev_theta_error);
        float offset_pid = offset_pid_loop(current_offset_error, prev_offset_error);
        float leftmotorcontrol = maxlimit(100, theta_pid - offset_pid);
        float rightmotorcontrol = maxlimit(100, -theta_pid - offset_pid);

        motorrotate(leftmotorcontrol, motor1);
        motorrotate(rightmotorcontrol, motor2);

        Serial.println("Rotate loop: current_theta_error - " + String(abs(current_theta_error)) + "; max_theta_error: "
                       + String(max_theta_error) + " delta_theta_left: " + String(delta_theta_left) +
                       "; delta_theta_right: " + String(delta_theta_right));

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
    Serial.println("exiting the loop");
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

// BELOW IS AN IMU APPROACH TO PID ANGLE CONTROL

/*
float integralSum = 0;
float lastError = 0;

float modWrapper(float degrees) {
  while(degrees > 180.0) {
    degrees -= 360;
  }
  while(degrees < -180) {
    degrees += 360;
  }
  return degrees;
}

float PIDControl(float reference, float state) {
  float error = modWrapper(reference - state);
  float integralSum += error * millis() * 1000;
  float derivative = (error - lastError) / (millis() * 1000);
  float lastError = error;
  float output = (error * Kp) + (derivative * Kd)  + (integralSum * Ki);
  return output;
}

void rover_rotate(float referenceAngle) {
  while (abs(lastError) > max_theta_error) {
    float power = PIDControl(referenceAngle, get_total_y(millis()));
    float leftmotorcontrol = maxlimit(100, power);
    float rightmotorcontrol = maxlimit(100, -power);
    Serial.println("IMU Experiments");
  }
} */