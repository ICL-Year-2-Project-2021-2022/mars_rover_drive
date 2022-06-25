#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "SensorFusion.h"
SF fusion;

float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
float pitch, roll, yaw;
float deltat;

Adafruit_MPU6050 mpu;

#define EULER_DATA
//#define RAW_DATA
//#define PROCESSING
//#define SERIAL_PLOTER

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
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

  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
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

  Serial.println("");
  delay(100);
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

#ifdef RAW_DATA
  Serial.print("From last Update:\t");
  Serial.println(deltat, 6);
  Serial.println("GYRO:\tx:"+ String(g.gyro.x) +"\t\ty:"+String(g.gyro.y)
         + "\t\tz:" + String(g.gyro.z));
  Serial.println("ACC:\tx:"+String(a.acceleration.x)+"\t\ty:"+String(a.acceleration.y)
         +"\t\tz:"+String(a.acceleration.z));
  Serial.println("TEMP:\t"+String(temp.temperature)+"\n");
#endif

  deltat = fusion.deltatUpdate();
  // fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //mahony
  // is suggested if there isn't the mag
  fusion.MahonyUpdate(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x,
                      a.acceleration.y, a.acceleration.z,
                      temp.temperature);  // else use the magwick

  roll = fusion.getRoll();
  pitch = fusion.getPitch();
  yaw = fusion.getYaw();

#ifdef EULER_DATA
  Serial.println("Pitch:\t" + String(pitch) + "\t\tRoll:\t" + String(roll) +
                 "\t\tYaw:\t" + String(yaw) + "\n");
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

  delay(200);  // for readability
}
