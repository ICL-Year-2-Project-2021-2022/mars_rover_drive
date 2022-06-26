#include <Optical_Flow.h>

// Chip select pin is set by set_optical_cs() default is left
int PIN_MOUSECAM_CS = PIN_MOUSECAM_CS_LEFT;

int delta_u_au_left;
int delta_v_au_left;
int delta_u_au_right;
int delta_v_au_right;

float delta_u_mm_left;
float delta_v_mm_left;
float delta_u_mm_right;
float delta_v_mm_right;

float total_u_left;
float total_v_left;
float total_u_right;
float total_v_right;

float reference_theta_left;
float delta_theta_left;
float reference_theta_right;
float delta_theta_right;

float total_theta_left;
float total_theta_right;

volatile byte movementflag = 0;
volatile int xydat[2];

int tdistance = 0;

void set_left_optical_cs(bool isLeft) {
  PIN_MOUSECAM_CS = isLeft ? PIN_MOUSECAM_CS_LEFT : PIN_MOUSECAM_CS_RIGHT;
}

int convTwosComp(int b) {
  // Convert from 2's complement
  if (b & 0x80) {
    b = -1 * ((b ^ 0xff) + 1);
  }
  return b;
}

// converts arbitrary values from sensor to mm using constant conversion factor
float convertDistanceToMM(int x, float au_2_mm) {
  return x / au_2_mm;
}

void mousecam_reset() {
  digitalWrite(PIN_MOUSECAM_RESET, HIGH);
  delay(1);  // reset pulse >10us
  digitalWrite(PIN_MOUSECAM_RESET, LOW);
  delay(35);  // 35ms from reset to functional
}

int mousecam_init() {
  pinMode(PIN_MOUSECAM_RESET, OUTPUT);
  pinMode(PIN_MOUSECAM_CS, OUTPUT);

  digitalWrite(PIN_MOUSECAM_CS, HIGH);

  mousecam_reset();
  return 1;
}

void mousecam_write_reg(int reg, int val) {
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(val);
  digitalWrite(PIN_MOUSECAM_CS, HIGH);
  delayMicroseconds(50);
}

int mousecam_read_reg(int reg) {
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg);
  delayMicroseconds(75);
  int ret = SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS, HIGH);
  delayMicroseconds(1);
  return ret;
}

void mousecam_read_motion(struct MD* p) {
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(ADNS3080_MOTION_BURST);
  delayMicroseconds(75);
  p->motion = SPI.transfer(0xff);
  p->dx = SPI.transfer(0xff);
  p->dy = SPI.transfer(0xff);
  p->squal = SPI.transfer(0xff);
  p->shutter = SPI.transfer(0xff) << 8;
  p->shutter |= SPI.transfer(0xff);
  p->max_pix = SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS, HIGH);
  delayMicroseconds(5);
}

float get_delta_theta(float delta_u_mm,
                      float delta_v_mm,
                      float sensor_displacement_local) {
  float reference_theta = acos(1 - (pow(delta_u_mm, 2) + pow(delta_v_mm, 2)) /
                                       (2 * pow(sensor_displacement_local, 2)));
  return (delta_u_mm < 0) ? reference_theta : -reference_theta;
}

// loop functions
void check_cumulative_dist() {

  MD md_left;
  MD md_right;

  set_left_optical_cs(true);
  mousecam_read_motion(&md_left);
  delay(1);

  set_left_optical_cs(false);
  mousecam_read_motion(&md_right);
  delay(1);
/*
  if (md_left.dy > 30 || md_right.dy > 30) {
    delay(1);
    check_cumulative_dist();
  }
*/
  // measured changes in r and l for left
  delta_u_au_left = convTwosComp(md_left.dx);
  delta_v_au_left = convTwosComp(md_left.dy);
  // measured changes in r and l for right
  delta_u_au_right = convTwosComp(md_right.dx);
  delta_v_au_right = convTwosComp(md_right.dy);

  // convert from measured changes in u and v for left
  delta_u_mm_left = convertDistanceToMM(delta_u_au_left, au_2_mm_left);
  delta_v_mm_left = convertDistanceToMM(delta_v_au_left, au_2_mm_left);
  // convert from measured changes in u and v for right
  delta_u_mm_right = convertDistanceToMM(delta_u_au_right, au_2_mm_right);
  delta_v_mm_right = convertDistanceToMM(delta_v_au_right, au_2_mm_right);

  // total distance (cumulative) for left
  total_u_left = total_u_left + delta_u_mm_left;
  total_v_left = total_v_left + delta_v_mm_left;
  // total distance (cumulative) for right
  total_u_right = total_u_right + delta_u_mm_right;
  total_v_right = total_v_right + delta_v_mm_right;

  // angle calculation for left and right
  /*reference_theta_left =
      acos(1 - (pow(delta_u_mm_left, 2) + pow(delta_v_mm_left, 2)) /
                   (2 * pow(sensor_displacement, 2)));
  delta_theta_left =
      (delta_v_mm_left > 0) ? reference_theta_left : -reference_theta_left;

  reference_theta_right =
      acos(1 - (pow(delta_u_mm_right, 2) + pow(delta_v_mm_right, 2)) /
                   (2 * pow(sensor_displacement, 2)));
  delta_theta_right =
      (delta_v_mm_right > 0) ? reference_theta_right : -reference_theta_right;
  */
  delta_theta_left =
      get_delta_theta(delta_u_mm_left, delta_v_mm_left, sensor_displacement);
  delta_theta_right =
      get_delta_theta(delta_u_mm_right, delta_v_mm_right, sensor_displacement);
  //Serial.println("deltatheta "+String(delta_theta_left) + "," + String(delta_theta_right));
  total_theta_left = total_theta_left + delta_theta_left;
  total_theta_right = total_theta_right + delta_theta_right;

  // Serial.println("(Total_Theta_Left, Total_Theta_Right):(" +
  // String(total_theta_left) + "," + String(total_theta_right) + ")");
  // Serial.println("(Delta_r,Delta_l):(" + String(delta_r) + "," +
  // String(delta_l) + ")"); Serial.print("Total_Thetha
  //: " + String(total_theta)); Serial.println("| Thetha: " +
  // String(delta_theta));
}

