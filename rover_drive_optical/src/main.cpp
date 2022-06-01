/*
Important to note that we use mm for distances and radians for angles
*/

#include <Arduino.h>
#include <Motor_Drive.h>
#include <Optical_Flow.h>
#include <SPI.h>
#include <math.h>

// max errors
const float max_dist_error = 0.01;
const float max_turn_error = 0.01;
const float max_theta_error = 0.01;

const int min_motor_val = 25;
const int max_motor_val = 100;

unsigned long last_print;

// loop functions

void check_cumulative_dist() {
  MD md_left;
  MD md_right;

  set_left_optical_cs(true);
  mousecam_read_motion(&md_left);
  delay(100);

  set_left_optical_cs(false);
  mousecam_read_motion(&md_right);
  delay(100);

  // measured changes in r and l for left
  delta_u_au_left = convTwosComp(md_left.dx);
  delta_v_au_left = convTwosComp(md_left.dy);
  // measured changes in r and l for right
  delta_u_au_right = convTwosComp(md_right.dx);
  delta_v_au_right = convTwosComp(md_right.dy);

  // convert from measured changes in u and v for left
  delta_u_mm_left = convertDistanceToMM(delta_u_au_left);
  delta_v_mm_left = convertDistanceToMM(delta_v_au_left);
  // convert from measured changes in u and v for right
  delta_u_mm_right = convertDistanceToMM(delta_u_au_right);
  delta_v_mm_right = convertDistanceToMM(delta_v_au_right);

  // total distance (cumulative) for left
  total_u_left = total_u_left + delta_u_mm_left;
  total_v_left = total_v_left + delta_v_mm_left;
  // total distance (cumulative) for right
  total_u_right = total_u_right + delta_u_mm_right;
  total_v_right = total_v_right + delta_v_mm_right;

  // Serial.print("(Total_Distance_r, Total_Distance_l):(" + String(total_r) +
  // "," + String(total_l) + ")|"); Serial.println("(Delta_r,Delta_l):(" +
  // String(delta_r) + "," + String(delta_l) + ")"); Serial.print("Total_Thetha
  // : " + String(total_theta)); Serial.println("| Thetha: " +
  // String(delta_theta));
}

/*
void angle_control(double theta_reqd) {
  double arc_length = 2*PI*abs(theta_reqd)*0.17/360; //0.17 is radius between
axle and sensor

  if (theta_reqd < 0) {
    while (){
     //move motor anticlockwise
    }
  }
  else {
    while (){
     //move motor clockwise
    }
  }
}
*/
bool enable_pid(float dist_reqd,
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
}

// limiting function
float maxlimit(float max, float input) {
  float out;
  if (input > max) {
    out = max;
  } else {
    out = input;
  }
  return out;
}

// motor profiling function ie sets limits for the minimum motor power
int motor_profile(int preadj_speed) {

  float adj_speed = 0;
  if (preadj_speed > 0) {
    adj_speed =
        (preadj_speed / 100.0) * (max_motor_val - min_motor_val) + min_motor_val;
  } else if (preadj_speed < 0) {
    adj_speed =
        (preadj_speed / 100.0) * (max_motor_val - min_motor_val) - min_motor_val;
  }
  return (int)adj_speed;
  /*return (x == 0) ? 0
         : (x > 0)
             ? (x / 100) * (max_motor_val - min_motor_val) + min_motor_val
             : (x / 100) * (max_motor_val - min_motor_val) - min_motor_val;*/
}

// motor function (to remove need for CCW and CW -> -100 to 100)
void motorrotate(int speed, int motor_no) {
  Serial.println(speed);
  Serial.println(motor_profile(speed));
  if (speed > 0) {
    robot.rotate(motor_no, motor_profile(speed), CCW);
  } else {
    robot.rotate(motor_no, motor_profile(abs(speed)), CW);
  }
}

// distance PD loop
float R_pid_loop(float dist_error, float prev_dist_error) {
  float dist_derivative = dist_error - prev_dist_error;
  float kp_dist = 0.2;
  float kd_dist = 0;
  float R_pid = kp_dist * dist_error + kd_dist * dist_derivative;
  R_pid = maxlimit(100, R_pid);
  return R_pid;
}

// angle PD loop
float theta_pid_loop(float theta_error, float prev_theta_error) {
  float theta_derivative = theta_error - prev_theta_error;
  float kp_theta = 20;
  float kd_theta = 10;
  float theta_pid = kp_theta * theta_error + kd_theta * theta_derivative;
  return theta_pid;
}

// turn PD loop
float turn_pid_loop(float turn_error, float prev_turn_error) {
  float turn_derivative = turn_error - prev_turn_error;
  float kp_turn = 0.0;
  float kd_turn = 0;
  float turn_pid = kp_turn * turn_error + kd_turn * turn_derivative;
  return turn_pid;
}

// motor control straight
// takes distance in mm
void motor_straight(float dist_reqd) {
  float current_dist_error = dist_reqd;
  float current_turn_error = 0;
  while (current_dist_error > max_dist_error) {
    check_cumulative_dist();
    float prev_dist_error = current_dist_error;
    current_dist_error =
        current_dist_error - (delta_v_mm_left + delta_v_mm_right) / 2;

    float prev_turn_error = current_turn_error;
    current_turn_error = delta_v_mm_right - delta_v_mm_left;
    // condition to exit loop
    if (current_dist_error < max_dist_error) {
      return;
    }

    float R_pid = R_pid_loop(current_dist_error, prev_dist_error);
    float turn_pid = turn_pid_loop(current_turn_error, prev_turn_error);

    float leftmotorcontrol = maxlimit(100, R_pid + turn_pid);
    float rightmotorcontrol = maxlimit(100, R_pid - turn_pid);

    motorrotate(leftmotorcontrol, motor1);
    motorrotate(rightmotorcontrol, motor2);

    if ((millis() - last_print) > 1000) {
      Serial.println("Current dist error " + String(current_dist_error) +
                     "Prev dist error " + String(prev_dist_error));
      Serial.println("Left motor control " + String(leftmotorcontrol) +
                     ", Right motor control " + String(rightmotorcontrol));
      Serial.println("\n");
      last_print = millis();
    }
  }
}

void motor_rotate(float theta_reqd) {}
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
  motor_straight(500);  // move 500mm units
  delay(3000);
  // motor_control(0, PI / 2); // probably need radians -> maybe we convert for
  // the commands
  // delay(3000);
}
