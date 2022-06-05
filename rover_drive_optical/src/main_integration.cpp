/*
Important to note that we use mm for distances and radians for angles
*/

#include <Arduino.h>
#include <Motor_Drive.h>
#include <Optical_Flow.h>
#include <SPI.h>
#include <math.h>

volatile bool drive_trigger;
volatile bool web_trigger;

// Inspired by https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
// and https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Timer/RepeatTimer/RepeatTimer.ino
hw_timer_t* timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  drive_trigger = true;
  // skips web connection function if interrupts occurs mid loop code.
  web_trigger = false;
  portEXIT_CRITICAL_ISR(&timerMux);
}
/*
struct velocities_class {
  float linear_velocity;
  float angular_velocity;
} velocities;

struct motors {
  int left_motor;
  int right_motor;
} motorcontrol;
*/
unsigned long last_print;

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

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);

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

/*void robot_move(float q_reqd, float p_reqd, float phi_reqd) {}
// contorls velocities
velocities_class outer_loop(float q_reqd, float p_reqd, float phi_reqd) {
  float velocity_reqd = distance_loop;
  float angular_velocity_reqd = theta_loop;

  return velocities;
}
// controls velocities
motors inner_loop(float velocity_reqd, float angular_velocity_reqd) {
  motorcontrol.left_motor = ;
  motorcontrol.right_motor = ;
  return motorcontrol;
}*/

void loop() {
  if (drive_trigger) {
    rover_move();
    update_directions();
    portENTER_CRITICAL_ISR(&timerMux);
    drive_trigger = false;
    web_trigger = true;
    portEXIT_CRITICAL_ISR(&timerMux);
  }
  if (web_trigger) {
    web_send();
    portENTER_CRITICAL_ISR(&timerMux);
    web_trigger = false;
    portEXIT_CRITICAL_ISR(&timerMux);
  }
}
