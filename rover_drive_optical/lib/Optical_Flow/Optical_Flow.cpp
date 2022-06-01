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
float convertDistanceToMM(int x) {
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