#ifndef IMU
#define IMU

#include <Adafruit_BNO08x.h>
#include <byte_conversions.ino>

#define BNO085_SDA
#define BNO085_SCL

#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

struct IMU {
  float accX;
  float accY;
  float accZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float magX;
  float magY;
  float magZ;
  float anVX;
  float anVY;
  float anVZ;
} imu;

// Here is where you define the sensor outputs you want to receive
void setIMUReports() {
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.write('M');
    Serial.println("Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.write('M');
    Serial.println("Could not enable gyroscope");
  }
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    Serial.write('M');
    Serial.println("Could not enable magnetic field calibrated");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
    Serial.write('M');
    Serial.println("Could not enable linear acceleration");
  }
  if (!bno08x.enableReport(SH2_GYRO_INTEGRATED_RV)) {
    Serial.write('M');
    Serial.println("Could not enable integrated gyroscope");
  }
  Serial.write('M');
  Serial.println("Set IMU reports");
  // if (!bno08x.enableReport(SH2_GRAVITY)) {
  //   Serial.println("Could not enable gravity vector");
  // }
  // if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
  //   Serial.println("Could not enable rotation vector");
  // }
  // if (!bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR)) {
  //   Serial.println("Could not enable geomagnetic rotation vector");
  // }
}

void readIMU() {
  if (bno08x.wasReset()) {
    setIMUReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  switch (sensorValue.sensorId) {
    case SH2_GYROSCOPE_CALIBRATED:
      imu.gyroX = sensorValue.un.gyroscope.x;
      imu.gyroY = sensorValue.un.gyroscope.y;
      imu.gyroZ = sensorValue.un.gyroscope.z;

      Serial.write('G');
      floatToBytes.value = imu.gyroX;
      Serial.write(floatToBytes.bytesArr, sizeof(float));
      floatToBytes.value = imu.gyroY;
      Serial.write(floatToBytes.bytesArr, sizeof(float));
      floatToBytes.value = imu.gyroZ;
      Serial.write(floatToBytes.bytesArr, sizeof(float));
      Serial.write('\n');

      return;
      break;

    case SH2_MAGNETIC_FIELD_CALIBRATED:
      imu.magX = sensorValue.un.magneticField.x;
      imu.magY = sensorValue.un.magneticField.y;
      imu.magZ = sensorValue.un.magneticField.z;

      Serial.write('M');
      floatToBytes.value = imu.magX;
      Serial.write(floatToBytes.bytesArr, sizeof(float));
      floatToBytes.value = imu.magY;
      Serial.write(floatToBytes.bytesArr, sizeof(float));
      floatToBytes.value = imu.magZ;
      Serial.write(floatToBytes.bytesArr, sizeof(float));
      Serial.write('\n');

      return;
      break;

    case SH2_LINEAR_ACCELERATION:
      imu.accX = sensorValue.un.linearAcceleration.x;
      imu.accY = sensorValue.un.linearAcceleration.y;
      imu.accZ = sensorValue.un.linearAcceleration.z;

      Serial.write('A');
      floatToBytes.value = imu.accX;
      Serial.write(floatToBytes.bytesArr, sizeof(float));
      floatToBytes.value = imu.accY;
      Serial.write(floatToBytes.bytesArr, sizeof(float));
      floatToBytes.value = imu.accZ;
      Serial.write(floatToBytes.bytesArr, sizeof(float));
      Serial.write('\n');

      return;
      break;

    // need to integrate twice to get rpy, not a good test
    // case SH2_GYRO_INTEGRATED_RV:
    //   imu.anVX = sensorValue.un.gyroIntegratedRV.x;
    //   imu.anVY = sensorValue.un.gyroIntegratedRV.y;
    //   imu.anVZ = sensorValue.un.gyroIntegratedRV.z;

    //   Serial.write('R');
    //   floatToBytes.value = imu.anVX;
    //   Serial.write(floatToBytes.bytesArr, sizeof(float));
    //   floatToBytes.value = imu.anVY;
    //   Serial.write(floatToBytes.bytesArr, sizeof(float));
    //   floatToBytes.value = imu.anVZ;
    //   Serial.write(floatToBytes.bytesArr, sizeof(float));
    //   Serial.write('\n');

    //   return;
  }
}

#endif