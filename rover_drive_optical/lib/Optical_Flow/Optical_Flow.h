#include <Arduino.h>
#include <SPI.h>

// optical flow defns
// these pins may be different on different boards

#define PIN_SS_LEFT 5    // D7
#define PIN_SS_RIGHT 22  // D3
#define PIN_MISO 19      // D5
#define PIN_MOSI 23      // D2
#define PIN_SCK 18       // D6

#define PIN_MOUSECAM_RESET 35     // A5
#define PIN_MOUSECAM_CS_LEFT 5    // D7
#define PIN_MOUSECAM_CS_RIGHT 22  // D3

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

// Chip select pin is set by set_optical_cs()
extern int PIN_MOUSCAM_CS;

const float au_2_mm_left = 4.24;
const float au_2_mm_right = 4.82;

// perpendicular distance from sensor to axis of rotation
const float sensor_displacement = 145;

extern int delta_u_au_left;
extern int delta_v_au_left;
extern int delta_u_au_right;
extern int delta_v_au_right;

extern float delta_u_mm_left;
extern float delta_v_mm_left;
extern float delta_u_mm_right;
extern float delta_v_mm_right;

extern float total_u_left;
extern float total_v_left;
extern float total_u_right;
extern float total_v_right;

extern float reference_theta_left;
extern float delta_theta_left;
extern float reference_theta_right;
extern float delta_theta_right;

extern float total_theta_left;
extern float total_theta_right;

extern volatile byte movementflag;
extern volatile int xydat[2];

extern int tdistance;

void set_left_optical_cs(bool isLeft);

int convTwosComp(int b);

float convertDistanceToMM(int x, float au_2_mm);

void mousecam_reset();

int mousecam_init();

void mousecam_write_reg(int reg, int val);

int mousecam_read_reg(int reg);

// Motion burst data from optical flow sensor
struct MD {
  byte motion;
  char dx, dy;
  byte squal;
  word shutter;
  byte max_pix;
};

void mousecam_read_motion(struct MD* p);

// pdata must point to an array of size ADNS3080_PIXELS_X x ADNS3080_PIXELS_Y
// you must call mousecam_reset() after this if you want to go back to normal
// operation
int mousecam_frame_capture(byte* pdata);

char asciiart(int k);
