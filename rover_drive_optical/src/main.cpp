#include <Arduino.h>
#include <LED_Strip.h>
#include <pid_loops.h>
#include <IMU.h>

void setup() {
  Serial.begin(9600);
  pinMode(PIN_SS_LEFT, OUTPUT);
  pinMode(PIN_SS_RIGHT, OUTPUT);
  pinMode(PIN_MISO, INPUT);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_SCK, OUTPUT);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);

  // motor setup
  robot.begin();

  // optical flow setup
  set_left_optical_cs(true);
  if (mousecam_init() == -1) {
    Serial.println("Left optical flow sensor failed to init");
    while (1)
      ;
  } else {
    Serial.println("Left optical flow sensor initialised.");
  }

  set_left_optical_cs(false);
  if (mousecam_init() == -1) {
    Serial.println("Right optical flow sensor failed to init");
    while (1)
      ;
  } else {
    Serial.println("Right optical flow sensor initialised.");
  }

  // imu setup
  imu_setup();
  if (mpu2.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  led_begin();
}

void loop() {
  rover_straight(300);
  delay(3000);
  rover_rotate(PI / 2);
  delay(3000);
}