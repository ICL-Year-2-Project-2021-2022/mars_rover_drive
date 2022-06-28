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

// float get_total_y(float time);
extern float deltat1;
extern float deltat2;

void imu_setup();

void check_imu_angle(float& delta_theta_left, float& delta_theta_right);
