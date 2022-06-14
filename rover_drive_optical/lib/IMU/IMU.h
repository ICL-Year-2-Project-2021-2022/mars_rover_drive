#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float rad_to_deg = 180/3.141592654;

float get_total_y(float time);