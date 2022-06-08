/*
Important to note that we use mm for distances and radians for angles
*/

#include <Arduino.h>
#include <Motor_Drive.h>
#include <Optical_Flow.h>
#include <SPI.h>
#include <math.h>

// max errors
const float max_dist_error = 0.05;
const float max_theta_error = 0.05;

unsigned long last_print;

/*bool enable_pid(float dist_reqd,
                float theta_reqd,
                float current_dist_error,
                float current_theta_error) {
  if (dist_reqd != 0 && theta_reqd == 0) {
    return abs(current_dist_error) > max_dist_error;
  } else if (dist_reqd == 0 && theta_reqd != 0) {
    return abs(current_theta_error) > max_theta_error;
  } else if (dist_reqd != 0 && theta_reqd != 0) {
    return abs(current_dist_error) > max_dist_error ||
           abs(current_theta_error) > max_theta_error;
  }
  return false;
}*/

// distance PD loop
float R_pid_loop(float dist_error, float prev_dist_error) {
  float dist_derivative = dist_error - prev_dist_error;
  float kp_dist = 2;
  float kd_dist = 0;
  float R_pid = kp_dist * dist_error + kd_dist * dist_derivative;
  R_pid = maxlimit(100, R_pid);
  return R_pid;
}

// angle PD loop
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
  float kp_turn = 0.5;
  float kd_turn = 0;
  float turn_pid = kp_turn * turn_error + kd_turn * turn_derivative;
  return turn_pid;
}

// offset PD loop
float offset_pid_loop(float offset_error, float prev_offset_error) {
  float offset_derivative = offset_error - prev_offset_error;
  float kp_offset = 0.5;
  float kd_offset = 0;
  float offset_pid = kp_offset * offset_error + kd_offset * offset_derivative;
  return offset_pid;
}

// motor control straight
// takes distance in mm
void rover_straight(float dist_reqd) {
  float current_dist_error = dist_reqd;
  float current_turn_error = 0;
  while (abs(current_dist_error) > max_dist_error) {
    check_cumulative_dist();
    float prev_dist_error = current_dist_error;
    current_dist_error =
        current_dist_error - (delta_v_mm_left + delta_v_mm_right) / 2;

    float prev_turn_error = current_turn_error;
    current_turn_error = delta_v_mm_right - delta_v_mm_left;
    // condition to exit loop
    if (abs(current_dist_error) < max_dist_error) {
      break;
    }

    float R_pid = R_pid_loop(current_dist_error, prev_dist_error);
    float turn_pid = turn_pid_loop(current_turn_error, prev_turn_error);

    float leftmotorcontrol = maxlimit(100, R_pid + turn_pid);
    float rightmotorcontrol = maxlimit(100, R_pid - turn_pid);

    motorrotate(leftmotorcontrol, motor1);
    motorrotate(rightmotorcontrol, motor2);

    /*if ((millis() - last_print) > 100) {
      Serial.println("Current dist error " + String(current_dist_error) +
                     ", Prev dist error " + String(prev_dist_error));
      Serial.println("Left motor control " + String(leftmotorcontrol) +
                     ", Right motor control " + String(rightmotorcontrol));
      Serial.println("\n");
      last_print = millis();
    }*/
  }
  robot.brake(motor1);
  robot.brake(motor2);
  //Serial.println("rover straight has finished");
  delay(50);
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
      break;
    }
    float theta_pid = theta_pid_loop(current_theta_error, prev_theta_error);
    float offset_pid = offset_pid_loop(current_offset_error, prev_offset_error);
    float leftmotorcontrol = maxlimit(100, theta_pid - offset_pid);
    float rightmotorcontrol = maxlimit(100, -theta_pid - offset_pid);

    motorrotate(leftmotorcontrol, motor1);
    motorrotate(rightmotorcontrol, motor2);

    Serial.println(String(total_theta_left) + "," + String(total_theta_right) +
                   "," + String(current_theta_error));

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

    delay(200);
  }
  robot.brake(motor1);
  robot.brake(motor2);
  // Serial.println("Theta pid finished");
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

void setup() {
  pinMode(PIN_SS_LEFT, OUTPUT);
  pinMode(PIN_SS_RIGHT, OUTPUT);
  pinMode(PIN_MISO, INPUT);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_SCK, OUTPUT);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);

  robot.begin();

  Serial.begin(9600);

  last_print = millis();

  set_left_optical_cs(true);
  if (mousecam_init() == -1) {
    Serial.println("Left optical flow sensor failed to init");
    while (1)
      ;
  }

  set_left_optical_cs(false);
  if (mousecam_init() == -1) {
    Serial.println("Right optical flow sensor failed to init");
    while (1)
      ;
  }
}

void loop() {
  // rover_straight(500);  // move 500mm units
  rover_straight(300);
  delay(3000);
  rover_straight(-300);
  delay(3000);
  //motorrotate(-10, motor1);
  //motorrotate(-10, motor2);
  //delay(5000);
  //robot.brake(motor1);
  //robot.brake(motor2);
  //delay(20000);
  // rover_rotate(PI / 2);
  // delay(3000);
  //   motor_control(0, PI / 2); // probably need radians -> maybe we convert
  //   for the commands delay(3000);
}
