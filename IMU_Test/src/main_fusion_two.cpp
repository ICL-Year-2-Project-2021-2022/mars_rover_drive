#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SensorFusion.h>
#include <Wire.h>
SF fusion;
SF fusion2;

float gx, gy, gz, ax, ay, az, temp;
float gx2, gy2, gz2, ax2, ay2, az2, temp2;
float pitch, roll, yaw;
float pitch2, roll2, yaw2;
float deltat;
float deltat2;

Adafruit_MPU6050 mpu;
Adafruit_MPU6050 mpu2;

#define EULER_DATA
//#define RAW_DATA
//#define PROCESSING
//#define SERIAL_PLOTER

void setup(void) {
  Serial.begin(9600);
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  //Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin(0x68)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  //Serial.println("MPU6050 Found!");

  //Serial.println("Adafruit MPU6050 2 test!");

  // Try to initialize!
  // ad0 must be connected to ground
  if (!mpu2.begin(0x69)) {
    Serial.println("Failed to find MPU6050 2 chip");
    while (1) {
      delay(10);
    }
  }
  //Serial.println("MPU6050 2 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  /*Serial.print("Accelerometer range set to: ");
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
  }*/
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  /*Serial.print("Gyro range set to: ");
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
  }*/

  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  /*Serial.print("Filter bandwidth set to: ");
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
  }*/

  //Serial.println("");

  delay(10);
  // Try to initialize!
  if (!mpu2.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  //Serial.println("MPU6050 Found!");

  mpu2.setAccelerometerRange(MPU6050_RANGE_4_G);
  /*Serial.print("Accelerometer range set to: ");
  switch (mpu2.getAccelerometerRange()) {
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
  }*/
  mpu2.setGyroRange(MPU6050_RANGE_500_DEG);
  /*Serial.print("Gyro range set to: ");
  switch (mpu2.getGyroRange()) {
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
  }*/

  mpu2.setFilterBandwidth(MPU6050_BAND_94_HZ);
  /*Serial.print("Filter bandwidth set to: ");
  switch (mpu2.getFilterBandwidth()) {
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
  }*/

  //Serial.println("");
  delay(100);
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Get new sensor events with the readings */
  sensors_event_t a2, g2, temp2;
  mpu2.getEvent(&a2, &g2, &temp2);

#ifdef RAW_DATA
  Serial << "From last Update:\t";
  Serial.println(deltat, 6);
  Serial << "GYRO:\tx:" << g.gyro.x << "\t\ty:" << g.gyro.y
         << "\t\tz:" << g.gyro.z << newl;
  Serial << "ACC:\tx:" << a.acceleration.x << "\t\ty:" << a.acceleration.y
         << "\t\tz:" << a.acceleration.z << newl;
  Serial << "TEMP:\t" << temp.temperature << newl << newl;
#endif
  // Serial.println(String(g.gyro.z) + "," + String(g2.gyro.z));
  deltat = fusion.deltatUpdate();
  deltat2 = fusion2.deltatUpdate();
  // fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //mahony
  // is suggested if there isn't the mag
  fusion.MahonyUpdate(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x,
                      a.acceleration.y, a.acceleration.z, deltat);
  fusion2.MahonyUpdate(g2.gyro.x, g2.gyro.y, g2.gyro.z, a2.acceleration.x,
                       a2.acceleration.y, a2.acceleration.z, deltat2);

  // fusion.MadgwickUpdate(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x,
  //                       a.acceleration.y, a.acceleration.z, deltat);

  roll = fusion.getRoll();
  pitch = fusion.getPitch();
  yaw = fusion.getYawRadians();
  roll2 = fusion2.getRoll();
  pitch2 = fusion2.getPitch();
  yaw2 = fusion2.getYawRadians();

#ifdef EULER_DATA
  // Serial.println("Pitch:\t" + String(pitch) + "\t\tRoll:\t" + String(roll) +
  //                "\t\tYaw:\t" + String(yaw) + "\n");
  //Serial.println("Yaw:\t" + String(yaw) + "\t\tYaw2:\t" + String(yaw2) +
  //               "\t\tYaw average:\t" + String((yaw2 + yaw) / 2));
  Serial.println(String(deltat)+","+String(deltat2)+","+String(yaw) + "," + String(yaw2) + "," + String((yaw2 + yaw) / 2));
#endif

#ifdef PROCESSING
  roll = fusion.getRollRadians();
  pitch = fusion.getPitchRadians();
  yaw = fusion.getYawRadians();
  Serial << pitch << ":" << roll << ":" << yaw << newl;
#endif

#ifdef SERIAL_PLOTER
  Serial << pitch << " " << roll << " " << yaw << endl;
#endif

  delay(20);  // for readability
}