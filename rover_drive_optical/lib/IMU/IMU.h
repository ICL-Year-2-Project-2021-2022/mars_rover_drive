#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

extern Adafruit_MPU6050 mpu2;

extern float rad_to_deg;

float get_total_y(float time);

void imu_setup();