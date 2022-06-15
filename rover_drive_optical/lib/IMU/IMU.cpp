#include <IMU.h>

Adafruit_MPU6050 mpu2;

float rad_to_deg = 180/3.141592654;

void imu_setup() {
  mpu2.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu2.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu2.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

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