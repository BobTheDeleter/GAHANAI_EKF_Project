#ifndef BYTES
#define BYTES

union FloatToBytes {
  float value;
  char bytesArr[sizeof(float)];
} floatToBytes;
union Uint64ToBytes {
  uint64_t value;
  char bytesArr[sizeof(uint64_t)];
} uint64ToBytes;
union DoubleToBytes {
  double value;
  char bytesArr[sizeof(double)];
} doubleToBytes;

#endif