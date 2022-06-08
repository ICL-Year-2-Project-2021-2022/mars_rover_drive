#include <Arduino.h>
#include "SPI.h"

// these pins may be different on different boards

#define PIN_SS_LEFT 5
#define PIN_SS_RIGHT 22
#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_SCK 18

#define PIN_MOUSECAM_RESET 35
#define PIN_MOUSECAM_CS_LEFT 5
#define PIN_MOUSECAM_CS_RIGHT 22

#define ADNS3080_PIXELS_X 30
#define ADNS3080_PIXELS_Y 30

#define ADNS3080_PRODUCT_ID 0x00
#define ADNS3080_REVISION_ID 0x01
#define ADNS3080_MOTION 0x02
#define ADNS3080_DELTA_X 0x03
#define ADNS3080_DELTA_Y 0x04
#define ADNS3080_SQUAL 0x05
#define ADNS3080_PIXEL_SUM 0x06
#define ADNS3080_MAXIMUM_PIXEL 0x07
#define ADNS3080_CONFIGURATION_BITS 0x0a
#define ADNS3080_EXTENDED_CONFIG 0x0b
#define ADNS3080_DATA_OUT_LOWER 0x0c
#define ADNS3080_DATA_OUT_UPPER 0x0d
#define ADNS3080_SHUTTER_LOWER 0x0e
#define ADNS3080_SHUTTER_UPPER 0x0f
#define ADNS3080_FRAME_PERIOD_LOWER 0x10
#define ADNS3080_FRAME_PERIOD_UPPER 0x11
#define ADNS3080_MOTION_CLEAR 0x12
#define ADNS3080_FRAME_CAPTURE 0x13
#define ADNS3080_SROM_ENABLE 0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER 0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER 0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER 0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER 0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER 0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER 0x1e
#define ADNS3080_SROM_ID 0x1f
#define ADNS3080_OBSERVATION 0x3d
#define ADNS3080_INVERSE_PRODUCT_ID 0x3f
#define ADNS3080_PIXEL_BURST 0x40
#define ADNS3080_MOTION_BURST 0x50
#define ADNS3080_SROM_LOAD 0x60

#define ADNS3080_PRODUCT_ID_VAL 0x17

// Chip select pin is set by set_optical_cs() default is left
int PIN_MOUSECAM_CS = PIN_MOUSECAM_CS_LEFT;

int total_x = 0;
int total_y = 0;

int total_x1_left = 0;
int total_y1_left = 0;
int total_x1_right = 0;
int total_y1_right = 0;

int x = 0;
int y = 0;

int a = 0;
int b = 0;

int distance_x_left = 0;
int distance_y_left = 0;
int distance_x_right = 0;
int distance_y_right = 0;

volatile byte movementflag = 0;
volatile int xydat[2];

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

int tdistance = 0;

void mousecam_reset() {
  digitalWrite(PIN_MOUSECAM_RESET, HIGH);
  delay(1);  // reset pulse >10us
  digitalWrite(PIN_MOUSECAM_RESET, LOW);
  delay(35);  // 35ms from reset to functional
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

int mousecam_init() {
  pinMode(PIN_MOUSECAM_RESET, OUTPUT);
  pinMode(PIN_MOUSECAM_CS, OUTPUT);

  digitalWrite(PIN_MOUSECAM_CS, HIGH);

  // change resolution
  /*uint8_t regVal = mousecam_read_reg(ADNS3080_CONFIGURATION_BITS);
  regVal |= 0x10;
  delayMicroseconds(50);
  mousecam_write_reg(ADNS3080_CONFIGURATION_BITS, (int)regVal);*/

  mousecam_reset();
  return 1;
}

struct MD {
  byte motion;
  char dx, dy;
  byte squal;
  word shutter;
  byte max_pix;
};

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

// pdata must point to an array of size ADNS3080_PIXELS_X x ADNS3080_PIXELS_Y
// you must call mousecam_reset() after this if you want to go back to normal
// operation
int mousecam_frame_capture(byte* pdata) {
  mousecam_write_reg(ADNS3080_FRAME_CAPTURE, 0x83);

  digitalWrite(PIN_MOUSECAM_CS, LOW);

  SPI.transfer(ADNS3080_PIXEL_BURST);
  delayMicroseconds(50);

  int pix;
  byte started = 0;
  int count;
  int timeout = 0;
  int ret = 0;
  for (count = 0; count < ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y;) {
    pix = SPI.transfer(0xff);
    delayMicroseconds(10);
    if (started == 0) {
      if (pix & 0x40)
        started = 1;
      else {
        timeout++;
        if (timeout == 100) {
          ret = -1;
          break;
        }
      }
    }
    if (started == 1) {
      pdata[count++] = (pix & 0x3f)
                       << 2;  // scale to normal grayscale byte range
    }
  }

  digitalWrite(PIN_MOUSECAM_CS, HIGH);
  delayMicroseconds(14);

  return ret;
}

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

  Serial.begin(57600);

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

char asciiart(int k) {
  static char foo[] = "WX86*3I>!;~:,`. ";
  return foo[k >> 4];
}

byte frame[ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y];

void loop() {
#if 0
  /*
      if(movementflag){

      tdistance = tdistance + convTwosComp(xydat[0]);
      Serial.println("Distance = " + String(tdistance));
      movementflag=0;
      delay(3);
      }

    */
  // if enabled this section grabs frames and outputs them as ascii art
  /*
    if(mousecam_frame_capture(frame)==0)
    {
      int i,j,k;
      for(i=0, k=0; i<ADNS3080_PIXELS_Y; i++)
      {
        for(j=0; j<ADNS3080_PIXELS_X; j++, k++)
        {
          //Serial.print(asciiart(frame[k]));
          Serial.print(frame[k]);
          Serial.print(' ');
        }
        Serial.println();
      }
    }
    Serial.println();
  */
  set_left_optical_cs(true);
  if (mousecam_frame_capture(frame) == 0) {
    Serial.println("A");
    int i, j, k;
    for (i = 0, k = 0; i < ADNS3080_PIXELS_Y; i++) {
      for (j = 0; j < ADNS3080_PIXELS_X; j++, k++) {
        Serial.println(frame[k]);
      }
    }
  }
  delay(20);

  set_left_optical_cs(false);
  if (mousecam_frame_capture(frame) == 0) {
    Serial.println("B");
    int i, j, k;
    for (i = 0, k = 0; i < ADNS3080_PIXELS_Y; i++) {
      for (j = 0; j < ADNS3080_PIXELS_X; j++, k++) {
        Serial.println(frame[k]);
      }
    }
  }
  delay(200);

#else

  // if enabled this section produces a bar graph of the surface quality that
  // can be used to focus the camera also drawn is the average pixel value 0-63
  // and the shutter speed and the motion dx,dy.

  // for(int i=0; i<md.squal/4; i++)
  //   Serial.print('*');
  // Serial.print(' ');
  // Serial.print((val*100)/351);
  // Serial.print(' ');
  // Serial.print(md.shutter); Serial.print(" (");
  // Serial.print((int)md.dx); Serial.print(',');
  // Serial.print((int)md.dy); Serial.println(')');

  // Serial.println(md.max_pix);

  MD md_left;
  MD md_right;

  set_left_optical_cs(true);
  mousecam_read_motion(&md_left);
  delay(10);

  set_left_optical_cs(false);
  mousecam_read_motion(&md_right);
  delay(10);

  distance_x_left = convTwosComp(md_left.dx);
  distance_y_left = convTwosComp(md_left.dy);

  distance_x_right = convTwosComp(md_right.dx);
  distance_y_right = convTwosComp(md_right.dy);

  total_x1_left = total_x1_left + distance_x_left;
  total_y1_left = total_y1_left + distance_y_left;

  total_x1_right = total_x1_right + distance_x_right;
  total_y1_right = total_y1_right + distance_y_right;

  // Serial.print('\n');
  Serial.print(md_left.squal);
  Serial.print(",");
  Serial.print(md_right.squal);
  Serial.print(",");
  // Serial.println("Distance_x = " + String(total_x));
  Serial.print(distance_x_left);
  Serial.print(",");
  Serial.print(distance_y_left);
  Serial.print(",");
  Serial.print(total_x1_left);
  Serial.print(",");
  Serial.print(total_y1_left);
  Serial.print(",");
  // Serial.println("Distance_y = " + String(total_y));
  // Serial.print('\n');
  Serial.print(distance_x_right);
  Serial.print(",");
  Serial.print(distance_y_right);
  Serial.print(",");
  Serial.print(total_x1_right);
  Serial.print(",");
  Serial.println(total_y1_right);

  delay(100);

#endif
}
