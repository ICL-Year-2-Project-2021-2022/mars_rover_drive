#include <IMU.h>

if (mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

Serial.println("MPU6050 Found!");

mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

total_z = 0

  float get_total_y(float time) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    //by Euler's formulae we get the following:
    //float acceleration_angle_x = atan((a.acceleration.y/16384.0)/sqrt(pow((a.acceleration.x/16384.0),2) + pow((a.acceleration.z/16384.0),2)))*rad_to_deg;
    //float acceleration_angle_y = atan(-1*(a.acceleration.x/16384.0)/sqrt(pow((a.acceleration.y/16384.0),2) + pow((a.acceleration.z/16384.0),2)))*rad_to_deg;
    //float gx = g.acceleration.x * rad_to_deg;
    //float gy = g.acceleration.y * rad_to_deg;
    //x axis angle
    //we apply a complementary filter to low pass the gryoscope and high pass the accelerometer
    //float total_x = 0.98 * (total_x + gx * time) + 0.02 * acceleration_angle_x;
    //float total_y = 0.98 * (total_y + gy * time) + 0.02 * acceleration_angle_y;
    float total_z = total_z + g.acceleration.z * rad_to_deg * time;
    return total_y;
  }