#include <Arduino.h>
#include <SPI.h>
#include <Optical_Flow.h>
#include <Motor_Drive.h>
#include <math.h>

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

  MD md;
  mousecam_read_motion(&md);
  delay(100);
  // measured changes in r and l
  distance_r_au = convTwosComp(md.dx);
  distance_l_au = convTwosComp(md.dy);
  // convert from measured changes in r and l 
  distance_r_mm = convertDistanceToMM(distance_r_au);
  distance_l_mm = convertDistanceToMM(distance_l_au);
  // total distance (cumulative)
  total_r = total_r + distance_r_mm;
  total_l = total_l + distance_l_mm;
  // scaled change in distance
  delta_r = distance_r_mm;
  delta_l = distance_l_mm;
  // change in angle, approximation using the cos rule
  float reference_theta = acos(1 - (pow(delta_l,2) + pow(delta_r,2)) / (2 * pow(sensor_displacement,2)));
  delta_theta = (delta_l > 0) ? reference_theta : - reference_theta; 
  // total theta (cumulative)                           
  total_theta = total_theta + delta_theta;
  
  Serial.print("(Total_Distance_r, Total_Distance_l):(" + String(total_r) + "," + String(total_l) +")|");
  Serial.println("(Delta_r,Delta_l):("+String(delta_r)+","+String(delta_l)+")");
  Serial.print("Total_Thetha : "+String(total_theta));
  Serial.println("| Thetha: "+String(delta_theta));

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
// limiting function
float maxlimit(float max, float input){
  float out;
  if(input > max){
    out = max;
  }
  else{
    out = input;
  }
  return out;
}

// motor function (to remove need for CCW and CW -> -100 to 100)
void motorrotate(int speed,int motor_no){
  if(speed>0){
    robot.rotate(motor_no, speed, CCW);
  }
  else{
    robot.rotate(motor_no,abs(speed),CW);
  }
}

// distance PD loop
float R_pid_loop(float dist_error, float prev_dist_error){
  //float dist_error = 0;
  //float prev_dist_error = 0;
  float dist_derivative = dist_error - prev_dist_error;
  float kp_dist = 3;
  float kd_dist = 0;
  float R_pid = kp_dist * dist_error + kd_dist * dist_derivative;
  R_pid = maxlimit(100,R_pid);
  return R_pid;
}

// angle PD loop
float theta_pid_loop(float theta_error, float prev_theta_error){
  //float theta_error = 0;
  //float prev_theta_error = 0;
  float theta_derivative = theta_error - prev_theta_error;
  float kp_theta = 0.5;
  float kd_theta = 0;
  //float theta_pid = 0;
  float theta_pid = kp_theta * theta_error + kd_theta * theta_derivative;
  return theta_pid;
}

/*float errors(float current_error, float previous_error, int delta){
  previous_error = current_error;
  current_error = current_error - delta;
  //return previous and current errors? how?
}*/

// main motor control function
void motor_control(float dist_reqd, float theta_reqd)
{
  float current_dist_error = dist_reqd;
  float current_theta_error = theta_reqd;
  while(pid_enable){
    check_cumulative_dist();
    float prev_dist_error = current_dist_error;
    current_dist_error = current_dist_error - delta_r;
    
    float prev_theta_error = current_theta_error;
    current_theta_error = current_theta_error - delta_theta;

    float R_pid = R_pid_loop(current_dist_error, prev_dist_error);
    float theta_pid = theta_pid_loop(current_theta_error, prev_theta_error);
    Serial.print("R_pid "+String(R_pid));
    Serial.println(", theta_pid "+String(theta_pid));

    float leftmotorcontrol = maxlimit(100, R_pid - theta_pid);
    float rightmotorcontrol = maxlimit(100, R_pid + theta_pid);
    Serial.println("Left motor control "+String(leftmotorcontrol)+", Right motor control "+String(rightmotorcontrol));
    Serial.println("\n");
    motorrotate(leftmotorcontrol, motor1);
    motorrotate(rightmotorcontrol, motor2);
    delay(1000);
  }
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

void loop(){
  motor_control(10,0); //move 10 units?
  //delay(3000);
  //motor_control(0,90); // probably need radians -> maybe we convert for the commands
}
/*void cumulative_loop()
{

  check_cumulative_dist();
  current_r = total_r;
  current_l = total_l;
  // angle_control(50); //degrees to rotate between -180 and +180
  //dist_control(20); // dist to travel (in cm)
  //delay(10000);
  check_cumulative_dist();
  current_r = total_r;
  current_l = total_l;
  // angle_control(50); //degrees to rotate between -180 and +180
  //dist_control(10); // dist to travel (in cm)
 // delay(10000);
}*/
