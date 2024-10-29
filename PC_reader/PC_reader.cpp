#include <ekfNavINS.h>
// changed certain functions to use std::unique_lock<std::shared_mutex> instead of std::unique_lock
// compiled ekfNavINS.cpp with PC_reader.cpp
#include <serialib.cpp>
#include <stdio.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <csignal>

#define SERIAL_PORT "COM5"
#define BAUD_RATE 115200
#define READ_BUFFER_SIZE 256
#define READ_TIMEOUT_U 2000

serialib serial;

char* readPtr;
char readBuffer[READ_BUFFER_SIZE];
int msgLen;

uint64_t iTOW;
bool iTOWFlag = false;
imuData imudata;
struct IntGyro {
    float x;
    float y;
    float z;
} intGyro;
gpsCoordinate gpscoordinate;
gpsVelocity gpsvelocity;

ekfNavINS ekf;
struct  Predictions {
    double lat;
    double lon;
    double alt;
    double vN;
    double vE;
    double vD;
    double roll;
    double pitch;
    double yaw;
} predictions;

char dataType;
union BytesToFloat {
    char bytes[sizeof(float)];
    float value;
} bytesToFloat;
union BytesToUint64 {
    char bytes[sizeof(uint64_t)];
    uint64_t value;
} bytesToUint64;
union BytesToDouble {
    char bytes[sizeof(double)];
    double value;
} bytesToDouble;

// int CK_A;
// int CK_B;
// void checksum(int size) {
//     CK_A = 0;
//     CK_B = 0;
    
//     for (int i = 0; i < size; i++)
//     {
//         CK_A += readBuffer[i];
//         CK_B += CK_A;
//     }
// }

#define OUTFILE "out.csv"
std::ofstream fout(OUTFILE);

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received. Exiting...\n";
    // Perform cleanup
    serial.closeDevice();
    fout.close();
    std::exit(signum);  // Use std::exit to terminate the program with the signal code
}

void openSerial() {
    // Connection to serial port
    char errorOpening = serial.openDevice(SERIAL_PORT, BAUD_RATE);
    if (errorOpening!=1) {
        std::cerr << "Unsuccessful connection to " << SERIAL_PORT << std::endl;
        std::exit(errorOpening);
    }
    std::cout << "Successful connection to " << SERIAL_PORT << std::endl;
}

void initialiseLogs() {
    // open log file
    if (fout.is_open()) {
        std::cout << "Successfully opened " << OUTFILE << std::endl;
    } else {
        std::cerr << "Could not open " << OUTFILE << std::endl;
        std::exit(2);
    }
    fout << "time,lat,lon,alt,vN,vE,vD,ekf_lat,ekf_lon,ekf_alt,ekf_roll,ekf_pitch,ekf_yaw,ekf_vN,ekf_vE,ekf_vD\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "time,lat,lon,alt,vN,vE,vD,ekf_lat,ekf_lon,ekf_alt,ekf_roll,ekf_pitch,ekf_yaw,ekf_vN,ekf_vE,ekf_vD\n";
    
}

void updatePredictions() {
    ekf.gpsCoordinateUpdateEKF(gpscoordinate);
    ekf.gpsVelocityUpdateEKF(gpsvelocity);
    ekf.imuUpdateEKF(iTOW, imudata);

    predictions.lat = ekf.getLatitude_rad();
    predictions.lon = ekf.getLongitude_rad();
    predictions.alt = ekf.getAltitude_m();
    predictions.vN = ekf.getVelNorth_ms();
    predictions.vE = ekf.getVelEast_ms();
    predictions.vD = ekf.getVelDown_ms();
    predictions.roll = ekf.getRoll_rad();
    predictions.pitch = ekf.getPitch_rad();
    predictions.yaw = ekf.getHeading_rad();
}

void logSensorsAndPredictions() {
    std::cout << iTOW
        <<","<< gpscoordinate.lat <<","<< gpscoordinate.lon <<","<< gpscoordinate.alt <<","<< gpsvelocity.vN <<","<< gpsvelocity.vE <<","<< gpsvelocity.vD
        <<","<< predictions.lat <<","<< predictions.lon <<","<< predictions.alt <<","<< predictions.vN <<","<< predictions.vE <<","<< predictions.vD <<","<< predictions.roll <<","<<  predictions.pitch <<","<< predictions.yaw
        << std::endl;
    
    fout << iTOW
        <<","<< gpscoordinate.lat <<","<< gpscoordinate.lon <<","<< gpscoordinate.alt <<","<< gpsvelocity.vN <<","<< gpsvelocity.vE <<","<< gpsvelocity.vD
        <<","<< predictions.lat <<","<< predictions.lon <<","<< predictions.alt <<","<< predictions.vN <<","<< predictions.vE <<","<< predictions.vD <<","<< predictions.roll <<","<<  predictions.pitch <<","<< predictions.yaw
        << std::endl;
}

void parseSensorData() {
    dataType = readBuffer[0];
    readPtr = readBuffer+1;
    switch (dataType)
    {
    case 'A':
        std::copy(readPtr, readPtr+sizeof(float), bytesToFloat.bytes);
        imudata.accX = bytesToFloat.value;
        readPtr += sizeof(float);

        // type_byte + 1st_float + 1, up to type_byte + 1st_float + 2nd_float
        std::copy(readPtr, readPtr+sizeof(float), bytesToFloat.bytes);
        imudata.accY = bytesToFloat.value;
        readPtr += sizeof(float);

        // type_byte + 1st_float + 2nd_float + 1, up to type_byte + 1st_float + 2nd_float + 3rd_float
        std::copy(readPtr, readPtr+sizeof(float), bytesToFloat.bytes);
        imudata.accZ = bytesToFloat.value;
        break;
    
    case 'G':
        std::copy(readPtr, readPtr+sizeof(float), bytesToFloat.bytes);
        imudata.gyroX = bytesToFloat.value;
        readPtr += sizeof(float);

        std::copy(readPtr, readPtr+sizeof(float), bytesToFloat.bytes);
        imudata.gyroY = bytesToFloat.value;
        readPtr += sizeof(float);

        std::copy(readPtr, readPtr+sizeof(float), bytesToFloat.bytes);
        imudata.gyroZ = bytesToFloat.value;
        break;
    
    case 'H':
        std::copy(readPtr, readPtr+sizeof(float), bytesToFloat.bytes);
        imudata.hX = bytesToFloat.value;
        readPtr += sizeof(float);

        std::copy(readPtr, readPtr+sizeof(float), bytesToFloat.bytes);
        imudata.hY = bytesToFloat.value;
        readPtr += sizeof(float);

        std::copy(readPtr, readPtr+sizeof(float), bytesToFloat.bytes);
        imudata.hZ = bytesToFloat.value;
        break;

    case 'P':
        iTOWFlag = 1;

        std::copy(readPtr, readPtr+sizeof(double), bytesToDouble.bytes);
        gpscoordinate.lat = bytesToDouble.value;
        readPtr += sizeof(double);

        std::copy(readPtr, readPtr+sizeof(double), bytesToDouble.bytes);
        gpscoordinate.lon = bytesToDouble.value;
        readPtr += sizeof(double);

        std::copy(readPtr, readPtr+sizeof(double), bytesToDouble.bytes);
        gpscoordinate.alt = bytesToDouble.value;
        readPtr += sizeof(double);

        std::copy(readPtr, readPtr+sizeof(uint64_t), bytesToUint64.bytes);
        iTOW = bytesToUint64.value;
        break;
    
    case 'V':
        iTOWFlag = 1;

        std::copy(readPtr, readPtr+sizeof(double), bytesToDouble.bytes);
        gpsvelocity.vN = bytesToDouble.value;
        readPtr += sizeof(double);

        std::copy(readPtr, readPtr+sizeof(double), bytesToDouble.bytes);
        gpsvelocity.vE = bytesToDouble.value;
        readPtr += sizeof(double);

        std::copy(readPtr, readPtr+sizeof(double), bytesToDouble.bytes);
        gpsvelocity.vD = bytesToDouble.value;
        readPtr += sizeof(double);

        std::copy(readPtr, readPtr+sizeof(uint64_t), bytesToUint64.bytes);
        iTOW = bytesToUint64.value;
        break;

    default:
        //std::cout << "Unrecognised data string" << std::endl;
        break;
    }
}

int main() {
    std::signal(SIGINT, signalHandler);
    openSerial();
    initialiseLogs();

    while (1) {
        msgLen = serial.readString(readBuffer, '\n', READ_BUFFER_SIZE, READ_TIMEOUT_U);
        // checksum(msgLen-4); // null, endl and 2 chks are not used in checksum
        // if (!(readBuffer[msgLen-4] == CK_A && readBuffer[msgLen-3] == CK_B)) {
        //     continue;// std::cout << "Incorrect checksum" << std::endl;
        // }
        parseSensorData();

        if (iTOWFlag) {
            updatePredictions();
            logSensorsAndPredictions();
            iTOWFlag = false;
        }
    }
    return 0;
}