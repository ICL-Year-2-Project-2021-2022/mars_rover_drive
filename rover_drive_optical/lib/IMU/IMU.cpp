#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <IMU.h>
#include <SensorFusion.h>
#include <Wire.h>
SF fusion;
SF fusion2;

Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;

float rad_to_deg = 180 / 3.141592654;

float gx, gy, gz, ax, ay, az, temp;
float gx2, gy2, gz2, ax2, ay2, az2, temp2;
float pitch, roll, yaw;
float pitch2, roll2, yaw2;
float current_yaw = 0.0;
float current_yaw2 = 0.0;
float deltat;
float deltat2;

void imu_setup() {
  mpu1.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu1.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu1.setFilterBandwidth(MPU6050_BAND_94_HZ);
  mpu2.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu2.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu2.setFilterBandwidth(MPU6050_BAND_94_HZ);
}

/*// eliminate drift by zeroing at the beginning of pid loop
void reset_imu_angle() {
  current_yaw = baseline_yaw2;
  current_yaw2 = baseline_yaw2;
}*/

// sensor fusion approach
void check_imu_angle(float& theta_left,
                     float& theta_right,
                     float& total_theta_left,
                     float& total_theta_right,
                     float& deltat) {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu1.getEvent(&a, &g, &temp);
  sensors_event_t a2, g2, temp2;
  mpu2.getEvent(&a2, &g2, &temp2);

  deltat = fusion.deltatUpdate();
  deltat2 = fusion2.deltatUpdate();
  // fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //mahony
  // is suggested if there isn't the mag
  fusion.MahonyUpdate(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x,
                      a.acceleration.y, a.acceleration.z, deltat);
  fusion2.MahonyUpdate(g2.gyro.x, g2.gyro.y, g2.gyro.z, a2.acceleration.x,
                       a2.acceleration.y, a2.acceleration.z, deltat2);

  // fusion.MadgwickUpdate(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x,
  //                      a.acceleration.y, a.acceleration.z, deltat);

  yaw = fusion.getYawRadians();
  yaw2 = fusion2.getYawRadians();

  float calculated_theta_left = ((yaw - current_yaw) + (yaw2 - current_yaw2)) / 2;
  float calculated_theta_right = ((yaw - current_yaw) + (yaw2 - current_yaw2)) / 2;
  if (calculated_theta_left > -0.8f && calculated_theta_left < 0.8f) {
      theta_left = calculated_theta_left;
      total_theta_left = total_theta_left + theta_left;
      current_yaw = total_theta_left;
  }
  if (calculated_theta_right > -0.8f && calculated_theta_right < 0.8f) {
      theta_right = calculated_theta_right;
      total_theta_right = total_theta_right + theta_right;
      current_yaw2 = total_theta_right;
  }
}

/*
float get_total_y(float time) {
  sensors_event_t a, g, temp;
  mpu2.getEvent(&a, &g, &temp);
  // by Euler's formulae we get the following:
  // float acceleration_angle_x =
  // atan((a.acceleration.y/16384.0)/sqrt(pow((a.acceleration.x/16384.0),2) +
  // pow((a.acceleration.z/16384.0),2)))*rad_to_deg; float acceleration_angle_y
  // = atan(-1*(a.acceleration.x/16384.0)/sqrt(pow((a.acceleration.y/16384.0),2)
  // + pow((a.acceleration.z/16384.0),2)))*rad_to_deg; float gx =
  // g.acceleration.x * rad_to_deg; float gy = g.acceleration.y * rad_to_deg; x
  // axis angle we apply a complementary filter to low pass the gryoscope and
  // high pass the accelerometer float total_x = 0.98 * (total_x + gx * time) +
  // 0.02 * acceleration_angle_x; float total_y = 0.98 * (total_y + gy * time) +
  // 0.02 * acceleration_angle_y;
  float total_z = total_z + g.acceleration.z * rad_to_deg * time;
  return total_z;
}

// sensor fusion approach
#if 1
void check_imu_angle(float& delta_theta_left,
                     float& delta_theta_right,
                     float& total_theta_left,
                     float& total_theta_right) {
  sensors_event_t a, g, temp;
  mpu2.getEvent(&a, &g, &temp);

  float deltat = fusion.deltatUpdate();
  // fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //mahony
  // is suggested if there isn't the mag
  fusion.MahonyUpdate(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x,
                      a.acceleration.y, a.acceleration.z,
                      deltat);  // else use the magwick

  // fusion.MadgwickUpdate(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x,
  //                      a.acceleration.y, a.acceleration.z, deltat);
  float yaw = fusion.getYaw();

  delta_theta_left = yaw - current_yaw;
  delta_theta_right = yaw - current_yaw;
  total_theta_left = yaw;
  total_theta_right = yaw;
}

// naive approach without sensor fusion. Only gyro integral
#else
void check_imu_angle(int& delta_theta_left,
                     int& delta_theta_right,
                     int& total_theta_left,
                     int& total_theta_right) {
  sensors_event_t a, g, temp;
  mpu2.getEvent(&a, &g, &temp);

  deltat = fusion.deltatUpdate();
  // fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //mahony
  // is suggested if there isn't the mag
  fusion.MahonyUpdate(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x,
                      a.acceleration.y, a.acceleration.z,
                      deltat);  // else use the magwick
  delta_theta_left = g.gyro.z * deltat;
  delta_theta_right = g.gyro.z * deltat;
  total_theta_left = total_theta_left + delta_theta_left;
  total_theta_right = total_theta_left + delta_theta_left;
}
#endif
