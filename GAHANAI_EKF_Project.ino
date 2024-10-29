#include <bno08x_read.ino>
#include <gps_read.ino>
#include <byte_conversions.ino>

#define SERIAL_BAUD 115200

#define GNSS_BAUD 9600
#define GPSRX 16
#define GPSTX 17

//ekfNavINS ekf;

void setup() {
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    return;
  }

  Serial.begin(SERIAL_BAUD);
  Serial2.begin(GNSS_BAUD, SERIAL_8N1, GPSRX, GPSTX);

  setIMUReports();
  setGPSReports();
}

const int GPS_MSG_LEN = 1+3*sizeof(double)+sizeof(uint64_t)+2+1;
const int IMU_MSG_LEN = 1+3*sizeof(float)+2+1;
char msg[GPS_MSG_LEN];

int A;
int B;
void checksum(char* arr, int size) {
  A = 0;
  B = 0;

  for (int i = 0; i < size; i++) {
    A += arr[i];
    B += A;
  }
}

GPSmsgType gpsMsgType;
IMUmsgType imuMsgType;

void loop() {
  imuMsgType = readIMU();
  // switch (imuMsgType) {
  //   case GYRO:
  //     msg[0] = 'G';
  //     floatToBytes.value = imu.gyroX;
  //     copy(floatToBytes.bytesArr, msg+1, sizeof(float));
  //     floatToBytes.value = imu.gyroY;
  //     copy(floatToBytes.bytesArr, msg+1+sizeof(float), sizeof(float));
  //     floatToBytes.value = imu.gyroZ;
  //     copy(floatToBytes.bytesArr, msg+1+2*sizeof(float), sizeof(float));
  //     checksum(msg, IMU_MSG_LEN);
  //     msg[IMU_MSG_LEN-3] = A;
  //     msg[IMU_MSG_LEN-2] = B;
  //     msg[IMU_MSG_LEN-1] = '\n';

  //     Serial.write(msg, IMU_MSG_LEN);
  //     break;
  //   case ACC:
  //     msg[0] = 'A';
  //     floatToBytes.value = imu.accX;
  //     copy(floatToBytes.bytesArr, msg+1, sizeof(float));
  //     floatToBytes.value = imu.accY;
  //     copy(floatToBytes.bytesArr, msg+1+sizeof(float), sizeof(float));
  //     floatToBytes.value = imu.accZ;
  //     copy(floatToBytes.bytesArr, msg+1+2*sizeof(float), sizeof(float));
  //     checksum(msg, IMU_MSG_LEN);
  //     msg[IMU_MSG_LEN-3] = A;
  //     msg[IMU_MSG_LEN-2] = B;
  //     msg[IMU_MSG_LEN-1] = '\n';

  //     Serial.write(msg, IMU_MSG_LEN);
  //     break;

  //   case MAG:
  //     msg[0] = 'M';
  //     floatToBytes.value = imu.magX;
  //     copy(floatToBytes.bytesArr, msg+1, sizeof(float));
  //     floatToBytes.value = imu.magY;
  //     copy(floatToBytes.bytesArr, msg+1+sizeof(float), sizeof(float));
  //     floatToBytes.value = imu.magZ;
  //     copy(floatToBytes.bytesArr, msg+1+2*sizeof(float), sizeof(float));
  //     checksum(msg, IMU_MSG_LEN);
  //     msg[IMU_MSG_LEN-3] = A;
  //     msg[IMU_MSG_LEN-2] = B;
  //     msg[IMU_MSG_LEN-1] = '\n';

  //     Serial.write(msg, IMU_MSG_LEN);
  //     break;
    
  //   default:
  //     break;
  // }

  gpsMsgType = readGPSFromSerial();
  // switch (gpsMsgType) {
  //   case VELNED:
  //     msg[0] = 'V';
  //     doubleToBytes.value = nav.velN;
  //     copy(doubleToBytes.bytesArr, msg+1, sizeof(double));
  //     doubleToBytes.value = nav.velE;
  //     copy(doubleToBytes.bytesArr, msg+1+sizeof(double), sizeof(double));
  //     doubleToBytes.value = nav.velD;
  //     copy(doubleToBytes.bytesArr, msg+1+2*sizeof(double), sizeof(double));
  //     uint64ToBytes.value = nav.iTOW;
  //     copy(doubleToBytes.bytesArr, msg+1+3*sizeof(double), sizeof(uint64_t));
  //     checksum(msg, GPS_MSG_LEN);
  //     msg[GPS_MSG_LEN-3] = A;
  //     msg[GPS_MSG_LEN-2] = B;
  //     msg[GPS_MSG_LEN-1] = '\n';

  //     Serial.write(msg, GPS_MSG_LEN);
  //     break;

  // case POSLLH:
  //     msg[0] = 'P';
  //     doubleToBytes.value = nav.lat;
  //     copy(doubleToBytes.bytesArr, msg+1, sizeof(double));
  //     doubleToBytes.value = nav.lng;
  //     copy(doubleToBytes.bytesArr, msg+1+sizeof(double), sizeof(double));
  //     doubleToBytes.value = nav.alt;
  //     copy(doubleToBytes.bytesArr, msg+1+2*sizeof(double), sizeof(double));
  //     uint64ToBytes.value = nav.iTOW;
  //     copy(uintToBytes.bytesArr, msg+1+3*sizeof(double), sizeof(uint64_t));
  //     checksum(msg, GPS_MSG_LEN);
  //     msg[GPS_MSG_LEN-3] = A;
  //     msg[GPS_MSG_LEN-2] = B;
  //     msg[GPS_MSG_LEN-1] = '\n';

  //     Serial.write(msg, GPS_MSG_LEN);
  //     break;

  //   default:
  //     break;
  // }
}

void copy(char* source, char* dest, int size) {
  for (int i = 0; i < size; i++) {
    dest[i] = source[i];
  }
}