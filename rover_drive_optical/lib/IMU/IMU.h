#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SensorFusion.h>
#include <Wire.h>

extern SF fusion;
extern SF fusion2;

extern float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
extern float pitch, roll, yaw;
extern float imu_deltat, imu_deltat2;

extern Adafruit_MPU6050 mpu1;
extern Adafruit_MPU6050 mpu2;

extern float current_yaw;
extern float current_yaw2;

extern float rad_to_deg;

// float get_total_y(float time);

// I2C uses SCL:IO22/D3,  SDA:IO21/D4

void imu_setup();

void check_imu_angle(float& theta_left,
                     float& theta_right,
                     float& total_theta_left_imu,
                     float& total_theta_right_imu);
