#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>

#define EEPROM_SIZE 12 // EEPROM SIZE DECLARATION - CONSTANT VAL

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

float total_x = 0;
float total_y = 0;

float total_x1_left = 0;
float total_y1_left = 0;
float total_x1_right = 0;
float total_y1_right = 0;

int x = 0;
int y = 0;

int a = 0;
int b = 0;

float distance_x_left = 0;
float distance_y_left = 0;
float distance_x_right = 0;
float distance_y_right = 0;

volatile byte movementflag = 0;
volatile int xydat[2];

//EEPROM 
int address_cal_value_left = 0;
int address_cal_value_right = 1;

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

float au_2_mm_left;
float au_2_mm_right;

void set_from_EEPROM() {
    au_2_mm_left = EEPROM.read(address_cal_value_left);
    au_2_mm_right = EEPROM.read(address_cal_value_right);
    Serial.println("AU_TO_MM_RIGHT: " + String(au_2_mm_right));
    Serial.println("AU_TO_MM_RIGHT: " + String(au_2_mm_left));
}

void calibrate_optical_sensors() {
    Serial.println("Starting Calibration. If previous values to be used, wait for 30 seconds or press 'N'. Otherwise press any other button.");
    unsigned long waitForThirty = millis();
    while (!Serial.available()) {
      delay(50);
        if (Serial.available() > 0) {   // read the incoming byte:
            Serial.println("PRINTING:" + String(Serial.available()));            
            char incomingChar = Serial.read();
            if (incomingChar == 'N' || incomingChar == 'n') {
                Serial.println("Calibraton aborted. Rover will be using past calibration values.");
                set_from_EEPROM();
                return; 
            }
            else {
                break;
            }
        }

        if ((millis() - waitForThirty) > 30000) {
            Serial.println("Timed-out. Rover will be using past calibration values.");
            set_from_EEPROM();
            return;
    }
    }

    Serial.println("Calibration process has started.");

    float cal_values_right[5];
    float cal_values_left[5];
    
    Serial.println("Move Rover FORWARD 50cm. Once completed, press any key.");

    while (!Serial.available()) {

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
        

    }
    char incomingChar = Serial.read();


    cal_values_left[0] = total_y1_left/500;
    cal_values_right[0] = total_y1_right/500;
    Serial.println("CAL VALUE LEFT: " + String(cal_values_left[0]));
    Serial.println("CAL VALUE RIGHT: " + String(cal_values_right[0]));
    distance_x_left = 0;
    distance_y_left = 0;
    distance_x_right = 0;
    distance_y_right = 0;
    total_x1_left = 0;
    total_y1_left = 0;
    total_x1_right = 0;
    total_y1_right = 0;

    Serial.println("Move Rover BACKWARDS 50cm. Once completed, press any key.");

    while (!Serial.available()) {

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

    }
    
    incomingChar = Serial.read();

    cal_values_left[1] = abs(total_y1_left)/500;
    cal_values_right[1] = abs(total_y1_right)/500;
    Serial.println("CAL VALUE LEFT: " + String(cal_values_left[1]));
    Serial.println("CAL VALUE RIGHT: " + String(cal_values_right[1]));
    distance_x_left = 0;
    distance_y_left = 0;
    distance_x_right = 0;
    distance_y_right = 0;
    total_x1_left = 0;
    total_y1_left = 0;
    total_x1_right = 0;
    total_y1_right = 0;

    Serial.println("Move Rover FORWARD 30cm. Once completed, press any key.");

    while (!Serial.available()) {

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

    }

    incomingChar = Serial.read();

    cal_values_left[2] = total_y1_left/300;
    cal_values_right[2] = total_y1_right/300;
    Serial.println("CAL VALUE LEFT: " + String(cal_values_left[2]));
    Serial.println("CAL VALUE RIGHT: " + String(cal_values_right[2]));
    distance_x_left = 0;
    distance_y_left = 0;
    distance_x_right = 0;
    distance_y_right = 0;
    total_x1_left = 0;
    total_y1_left = 0;
    total_x1_right = 0;
    total_y1_right = 0;

    Serial.println("Move Rover BACKWARDS 30cm. Once completed, press any key.");

    while (!Serial.available()) {

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

    }

    incomingChar = Serial.read();

    cal_values_left[3] = abs(total_y1_left)/300;
    cal_values_right[3] = abs(total_y1_right)/300;
    Serial.println("CAL VALUE LEFT: " + String(cal_values_left[3]));
    Serial.println("CAL VALUE RIGHT: " + String(cal_values_right[3]));
    distance_x_left = 0;
    distance_y_left = 0;
    distance_x_right = 0;
    distance_y_right = 0;
    total_x1_left = 0;
    total_y1_left = 0;
    total_x1_right = 0;
    total_y1_right = 0;

    Serial.println("Move Rover FORWARD 10cm. Once completed, press any key.");

    while (!Serial.available()) {

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

    }

    incomingChar = Serial.read();

    cal_values_left[4] = total_y1_left/100;
    cal_values_right[4] = total_y1_right/100;
    Serial.println("CAL VALUE LEFT: " + String(cal_values_left[4]));
    Serial.println("CAL VALUE RIGHT: " + String(cal_values_right[4]));
    distance_x_left = 0;
    distance_y_left = 0;
    distance_x_right = 0;
    distance_y_right = 0;
    total_x1_left = 0;
    total_y1_left = 0;
    total_x1_right = 0;
    total_y1_right = 0;

    Serial.println("Move Rover BACKWARDS 10cm. Once completed, press any key.");

    while (!Serial.available()) {

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

    }

    incomingChar = Serial.read();

    cal_values_left[5] = abs(total_y1_left)/100;
    cal_values_right[5] = abs(total_y1_right)/100;
    Serial.println("CAL VALUE LEFT: " + String(cal_values_left[5]));
    Serial.println("CAL VALUE RIGHT: " + String(cal_values_right[5]));
    distance_x_left = 0;
    distance_y_left = 0;
    distance_x_right = 0;
    distance_y_right = 0;
    total_x1_left = 0;
    total_y1_left = 0;
    total_x1_right = 0;
    total_y1_right = 0;

    float array_left_sum = 0;
    float array_right_sum = 0;

    for (int i=0;i < 6;i++){
        array_left_sum += cal_values_left[i];
        array_right_sum += cal_values_right[i];
    }
    
    float average_right_values = array_right_sum/6;
    float average_left_values = array_left_sum/6;

    Serial.println("Right Calibration value attained: " + String(average_right_values));
    Serial.println("Left Calibration value attained: " + String(average_left_values));

    Serial.println("If Calibration Values are fine, press 'Y', if not to repeat Calibration, press 'N.'");

    bool validResponse = false;

    while (!Serial.available() || validResponse == false) {
        if (Serial.available() > 0) {   // read the incoming byte:
            char incomingChar = Serial.read();
            if (incomingChar == 'N' || incomingChar == 'n') {
                calibrate_optical_sensors();
                validResponse = true;
            }
            else if (incomingChar == 'Y' || incomingChar == 'y') {
                EEPROM.write(address_cal_value_left, average_left_values);
                EEPROM.write(address_cal_value_right,average_right_values);
                Serial.println("Written for left: " + String(EEPROM.read(address_cal_value_left)));
                validResponse = true;
            }
            else {
                Serial.println("No such option. Try again.");
            }
        }
    }

    Serial.println("Calibration has ended.");

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

  //Init EEPROM
  EEPROM.begin(EEPROM_SIZE);

  Serial.begin(9600);

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

  calibrate_optical_sensors();
}

char asciiart(int k) {
  static char foo[] = "WX86*3I>!;~:,`. ";
  return foo[k >> 4];
}

byte frame[ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y];

void loop() {

}
