#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SensorFusion.h>
#include <Wire.h>

extern float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
extern float pitch, roll, yaw;
extern float deltat;

extern Adafruit_MPU6050 mpu2;

extern float current_yaw;

extern float rad_to_deg;

float get_total_y(float time);

void imu_setup();

void check_imu_angle(float& theta_left,
                           float& theta_right,
                           float& total_theta_left,
                           float& total_theta_right);