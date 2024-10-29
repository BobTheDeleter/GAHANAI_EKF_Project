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

void loop() {
  readIMU();
  readGPSFromSerial();
}

void copy(char* source, char* dest, int size) {
  for (int i = 0; i < size; i++) {
    dest[i] = source[i];
  }
}