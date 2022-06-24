#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SensorFusion.h>
#include <Wire.h>

extern Adafruit_MPU6050 mpu1;
extern Adafruit_MPU6050 mpu2;

extern float current_yaw;

extern float rad_to_deg;

// float get_total_y(float time);

void imu_setup();

void reset_imu_angle();

void check_imu_angle(float& theta_left,
                     float& theta_right,
                     float& total_theta_left,
                     float& total_theta_right);