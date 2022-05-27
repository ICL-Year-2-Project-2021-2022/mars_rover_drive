#include <Arduino.h>
#include <SPI.h>
#include <Optical_Flow.h>
#include <Motor_Drive.h>

// loop functions

void check_cumulative_dist()
{
#if 0

  if(mousecam_frame_capture(frame)==0)
  {
    int i,j,k;
    for(i=0, k=0; i<ADNS3080_PIXELS_Y; i++)
    {
      for(j=0; j<ADNS3080_PIXELS_X; j++, k++)
      {
        Serial.print(asciiart(frame[k]));
        Serial.print(' ');
      }
      Serial.println();
    }
  }
  Serial.println();
  delay(250);

#else

  // if enabled this section produces a bar graph of the surface quality that can be used to focus the camera
  // also drawn is the average pixel value 0-63 and the shutter speed and the motion dx,dy.

  int val = mousecam_read_reg(ADNS3080_PIXEL_SUM);
  MD md;
  mousecam_read_motion(&md);
  for (int i = 0; i < md.squal / 4; i++)
    Serial.print('*');
  Serial.print(' ');
  Serial.print((val * 100) / 351);
  Serial.print(' ');
  Serial.print(md.shutter);
  Serial.print(" (");
  Serial.print((int)md.dx);
  Serial.print(',');
  Serial.print((int)md.dy);
  Serial.println(')');

  // Serial.println(md.max_pix);
  delay(100);

  distance_x = md.dx; // convTwosComp(md.dx);
  distance_y = md.dy; // convTwosComp(md.dy);

  total_x1 = total_x1 + distance_x;
  total_y1 = total_y1 + distance_y;

  total_x = total_x1 / 157;
  total_y = total_y1 / 157;

  Serial.print('\n');

  Serial.println("Distance_x = " + String(total_x));

  Serial.println("Distance_y = " + String(total_y));
  Serial.print('\n');

#endif
}

/*
void angle_control(double theta_reqd) {
  double arc_length = 2*PI*abs(theta_reqd)*0.17/360; //0.17 is radius between axle and sensor

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
int maxlimit(int max, int input){
  int out;
  if(input > max){
    out = max;
  }
  else{
    out = input;
  }
  return out;
}

void motorrotate(int speed,int motor_no){
  if(speed>0){
    robot.rotate(motor_no, speed, CCW);
  }
  else{
    robot.rotate(motor_no,abs(speed),CW);
  }
}
dist_error = dist_reqd -
R_pid = R_pid_loop()
leftmotorcontrol = maxlimit(100, R_pid + theta_pid);
rightmotorcontrol = maxlimit(100, R_pid - theta_pid);

motorrotate(leftmotorcontrol, motor1);
motorrotate(rightmotorcontrol, motor2);

float R_pid_loop(){
  float dist_error = 0;
  float prev_dist_error = 0;
  float dist_derivative = dist_error - prev_dist_error;
  float kp_dist = 0;
  float kd_dist = 0;
  float R_pid = 0;
  R_pid = kp_dist * dist_error + kd_dist * dist_derivative;
  R_pid = maxlimit(100,R_pid);
  return R_pid;
}

float theta_pid_loop(){
  float theta_error = 0;
  float prev_theta_error = 0;
  float theta_derivative = theta_error - prev_theta_error;
  float kp_theta = 0;
  float kd_theta = 0;
  float theta_pid = 0;
  theta_pid = kp_theta * theta_error + kd_theta * theta_derivative;
  return theta_pid;
}


void dist_control(int dist_reqd)
{

  if (dist_reqd > 0)
  {
    while ((total_y - offset_y) < dist_reqd)
    {
      // move motor forwards
      robot.rotate(motor1, 100, CCW); // run motor1 at 60% speed in CW direction
      robot.rotate(motor2, 100, CW);  // run motor1 at 60% speed in CW direction
      delay(100);
      check_cumulative_dist();
      int error_x = total_x - offset_x;
      if (error_x != 0)
      {
        while (error_x != 0)
        {
          if (error_x > 0)
          {
            // move motors slightly left
            robot.rotate(motor1, 100, CCW); // run motor1 at 60% speed in CW direction
            robot.rotate(motor2, 100, CCW); // run motor1 at 60% speed in CW direction
            delay(10);                      // change to desired amount of turning
            break;
          }
          else
          {
            robot.rotate(motor1, 100, CW); // run motor1 at 60% speed in CW direction
            robot.rotate(motor2, 100, CW); // run motor1 at 60% speed in CW direction
            delay(10);                     // change to desired amount of turning
            break;
          }
        }
      }
    }
  }
  else
  {
    while ((total_y - offset_y) < -abs(dist_reqd))
    {
      // move motor backwards
      robot.rotate(motor1, 100, CW);  // run motor1 at 60% speed in CW direction
      robot.rotate(motor2, 100, CCW); // run motor1 at 60% speed in CW direction
      check_cumulative_dist();
      int error_x = total_x - offset_x;
      if (error_x != 0)
      {
        while (error_x != 0)
        {
          if (error_x > 0)
          {
            // move motors slightly left
            robot.rotate(motor1, 100, CW); // run motor1 at 60% speed in CW direction
            robot.rotate(motor2, 100, CW); // run motor1 at 60% speed in CW direction
            delay(10);                     // change to desired amount of turning
            break;
          }
          else
          {
            robot.rotate(motor1, 100, CCW); // run motor1 at 60% speed in CW direction
            robot.rotate(motor2, 100, CCW); // run motor1 at 60% speed in CW direction
            delay(10);                      // change to desired amount of turning
            break;
          }
        }
      }
    }
  }
  robot.brake(1);
  robot.brake(2);
}

void setup()
{
  pinMode(PIN_SS, OUTPUT);
  pinMode(PIN_MISO, INPUT);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_SCK, OUTPUT);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);

  robot.begin();

  Serial.begin(9600);

  if (mousecam_init() == -1)
  {
    Serial.println("Mouse cam failed to init");
    while (1)
      ;
  }
}

void loop()
{

  check_cumulative_dist();
  offset_x = total_x;
  offset_y = total_y;
  // angle_control(50); //degrees to rotate between -180 and +180
  dist_control(20); // dist to travel (in cm)
  delay(10000);
  check_cumulative_dist();
  offset_x = total_x;
  offset_y = total_y;
  // angle_control(50); //degrees to rotate between -180 and +180
  dist_control(10); // dist to travel (in cm)
  delay(10000);
}
