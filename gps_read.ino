#ifndef GPS
#define GPS

#include <byte_conversions.ino>

//format hex command strings for sending to UBLOX
//could switch to just nav-pvt for both llh and vel
const static char velned[] = {0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1e, 0x67};
const static char posllh[] = {0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0e, 0x47};
const static char sol[] = {0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x06, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x17, 0xDa};
const static char airborne[] = {0xb5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xff, 0xff, 0x08, 0x02, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
                                0x00, 0x00, 0x05, 0x00, 0xfa, 0x00, 0xfa, 0x00, 0x64, 0x00, 0x2c, 0x01, 0x00, 0x3c, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x53, 0x0b, 0xb5, 0x62, 0x06, 0x24,
                                0x00, 0x00, 0x2a, 0x84
                               };
const static char rate_a[] = {0xb5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x90, 0x01, 0x01, 0x00, 0x01, 0x00, 0xa7, 0x1f}; // 2.5 Hz
const static char rate_b[] = {0xb5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0e, 0x30}; // 2.5 Hz

#define GPS_INFO_BUFFER_SIZE 60 // enough for both the strings (SOL is only 52 long)
uint8_t GPS_info_buffer[GPS_INFO_BUFFER_SIZE];
unsigned string_length = 58;  //VELNED length is 42, POSLLH length is 34, SOL length is 58. Initialize with longest string.
unsigned int received_char;
int i = 0; // counter
bool message_started = false;

uint64_t iTOW; // time of week
long velN; // speed in centimeter / second
long velE; // speed in centimeter / second
long velD; // speed in centimeter / second
// unsigned long velocity_3d; // speed in centimeter / second
// float spd_kts; //speed in knots
long longitude ;//longitude
long latitude ;//latitude
long height_above_ellipsoid;//height above main sea level
// unsigned long ground_speed ;
// float heading;

//CHECKSUM CALCULATION VARIABLE
unsigned char CK_A = 0;
unsigned char CK_B = 0;

// byte sendlat = 0; // toggle to send either latitude or longitude in same field
char fixok = 0; // flag to indicate a valid GPS fix was received.

struct NAV {
  // posllh
  // unsigned char cls_posllh;
  // unsigned char id_posllh;
  // unsigned short len_posllh;
  uint64_t iTOW;
  // long lon;
  // long lat;
  // long lat_lon;
  // long height;
  // long hMSL;
  double lat;
  double lng;
  double alt;
  // unsigned long hAcc;
  // unsigned long vAcc;
  // velned
  // unsigned char cls_velned;
  // unsigned char id_velned;
  // unsigned short len_velned;
  // unsigned long iTOW_velned; // U4
  double velN; // I4
  double velE;
  double velD;
  // unsigned long speed; // Spd (3D)
  // unsigned long gSpeed;
  // long heading;
  // unsigned long sAcc;
} nav;

//TinyGPSPlus gps;

//Turns off NMEA sentences. Turns on UBX messages
void setGPSReports() {
  delay(100);
  Serial2.println("$PUBX,40,RMC,0,0,0,0*47"); //RMC OFF
  delay(100);
  Serial2.println("$PUBX,40,VTG,0,0,0,0*5E"); //VTG OFF
  delay(100);
  // not using NMEA sentence for LLA
  Serial2.println("$PUBX,40,GGA,0,0,0,0*5A"); //GGA OFF
  delay(100);
  Serial2.println("$PUBX,40,GSA,0,0,0,0*4E"); //GSA OFF
  delay(100);
  Serial2.println("$PUBX,40,GSV,0,0,0,0*59"); //GSV OFF
  delay(100);
  Serial2.println("$PUBX,40,GLL,0,0,0,0*5C"); //GLL OFF
  delay(100);

  Serial2.write(airborne, sizeof(airborne)); //set GPS mode to airborne < 4g
  delay(100);
  Serial2.write(rate_a, sizeof(rate_a)); //set GPS update rate to 4Hz (1st string)
  delay(100);
  Serial2.write(rate_b, sizeof(rate_b)); //set GPS update rate to 4Hz (2nd string)
  delay(100);
  Serial2.write(velned, sizeof(velned)); // VELNED (VELocity, North, East, Down) message ON
  // not using NMEA sentence for LLA
  delay(100);
  Serial2.write(posllh, sizeof(posllh)); // POSLLH (POSition Latitude Longitude Height) message ON
  delay(100);
  Serial2.write(sol, sizeof(sol)); // POSLLH (POSition Latitude Longitude Height) message ON

  //Serial.println("Set GPS reports");
} // end setup

// CALCULATES THE UBX CHECKSUM OF A CHAR ARRAY
void UBX_calculate_checksum(uint8_t* array, unsigned int array_start, unsigned int array_length) {
  CK_A = 0;
  CK_B = 0;
  unsigned int i;
  for (i = 0; i < array_length; i++) {
    CK_A = CK_A + array[array_start + i];
    CK_B = CK_B + CK_A;
  }
}

enum GPSmsgType {
  VELNED,
  POSLLH,
  NOGPS
};

GPSmsgType readGPSFromSerial() {
  if (Serial2.available()) { // If GPS data is available
    byte GPS_info_char = Serial2.read();  // Read it and store as HEX char

    if (GPS_info_char == 0xb5) { // ublox B5 record found
      message_started = true;
      received_char = 0;
    } 
    
    else if (message_started == true) { // the message is already started and I got a new character
      if (received_char < string_length) { // buffer not full
        GPS_info_buffer[received_char] = GPS_info_char;
        if (GPS_info_buffer[2] == 0x12) string_length = 42;  //VELNED lenght is 36 + 6
        if (GPS_info_buffer[2] == 0x02) string_length = 34;  //POSLLH lenght is 28 + 6
        if (GPS_info_buffer[2] == 0x06) string_length = 58;  //SOL lenght is 52 + 6
        received_char++;
      } // end buffer not full
      else { // final character received
        GPS_info_buffer[received_char] = GPS_info_char;
        UBX_calculate_checksum(GPS_info_buffer, 1, (string_length - 2)); // calculates checksum

        switch (string_length) {
          case 42: // VELNED string received
            if (CK_A == GPS_info_buffer[41] && CK_B == GPS_info_buffer[42]) { // confirm received and calculated checksums are the same
              // convert the GPS buffer array bytes 5,6,7,8 into one "unsigned long" (4 byte) field
              iTOW   = (long)GPS_info_buffer[8] << 24;
              iTOW  += (long)GPS_info_buffer[7] << 16;
              iTOW  += (long)GPS_info_buffer[6] << 8;
              iTOW  += (long)GPS_info_buffer[5];
              nav.iTOW = iTOW;

              // convert the GPS buffer array bytes 9,10,11,12 into one "unsigned long" (4 byte) field
              velN   = (long)GPS_info_buffer[12] << 24;
              velN  += (long)GPS_info_buffer[11] << 16;
              velN  += (long)GPS_info_buffer[10] << 8;
              velN  += (long)GPS_info_buffer[9];
              nav.velN = velN*1e-2;
              //Serial.print(" velN: "); Serial.println(nav.velN * 1e-2f);
              // convert the GPS buffer array bytes 13,14,15,16 into one "unsigned long" (4 byte) field
              velE   = (long)GPS_info_buffer[16] << 24;
              velE  += (long)GPS_info_buffer[15] << 16;
              velE  += (long)GPS_info_buffer[14] << 8;
              velE  += (long)GPS_info_buffer[13];
              nav.velE = velE*1e-2;
              //Serial.print(" velE: "); Serial.println(nav.velE * 1e-2f);
              // convert the GPS buffer array bytes 17,18,19,20 into one "unsigned long" (4 byte) field
              velD   = (long)GPS_info_buffer[20] << 24;
              velD  += (long)GPS_info_buffer[19] << 16;
              velD  += (long)GPS_info_buffer[18] << 8;
              velD  += (long)GPS_info_buffer[17];
              nav.velD = velD*1e-2;
              //Serial.println(nav.velD);

              Serial.print(" velD: "); Serial.println(nav.velD * 1e-2f);
              Serial.write('V');
              doubleToBytes.value = nav.velN;
              Serial.write(doubleToBytes.bytesArr, sizeof(double));
              doubleToBytes.value = nav.velE;
              Serial.write(doubleToBytes.bytesArr, sizeof(double));
              doubleToBytes.value = nav.velD;
              Serial.write(doubleToBytes.bytesArr, sizeof(double));
              uint64ToBytes.value = nav.iTOW;
              Serial.write(uint64ToBytes.bytesArr, sizeof(uint64_t));
              Serial.write('\n');
              return VELNED;
            }
            else {// when checksum error set speed = 0
              // spd_kts = 0;
              // ground_speed = 0;
              // heading = 0;
            }

            break;
          
          // Not using NMEA sentences for LLA
          case 34: //POSLLH string received
            if (CK_A == GPS_info_buffer[33] && CK_B == GPS_info_buffer[34]) { // confirm received and calculated checksums are the same
              iTOW   = (long)GPS_info_buffer[8] << 24;
              iTOW  += (long)GPS_info_buffer[7] << 16;
              iTOW  += (long)GPS_info_buffer[6] << 8;
              iTOW  += (long)GPS_info_buffer[5];
              nav.iTOW = iTOW;
              
              // convert the GPS buffer array bytes into one "long" (4 byte) field
              longitude   = (long)GPS_info_buffer[12] << 24;
              longitude  += (long)GPS_info_buffer[11] << 16;
              longitude  += (long)GPS_info_buffer[10] << 8;
              longitude  += (long)GPS_info_buffer[9];

              latitude   = (long)GPS_info_buffer[16] << 24;
              latitude  += (long)GPS_info_buffer[15] << 16;
              latitude  += (long)GPS_info_buffer[14] << 8;
              latitude  += (long)GPS_info_buffer[13];

              height_above_ellipsoid   = (long)GPS_info_buffer[20] << 24;
              height_above_ellipsoid  += (long)GPS_info_buffer[19] << 16;
              height_above_ellipsoid  += (long)GPS_info_buffer[18] << 8;
              height_above_ellipsoid  += (long)GPS_info_buffer[17];

              nav.lat = latitude*1e-7*PI/180;
              nav.lng = longitude*1e-7*PI/180;
              nav.alt = (double)height_above_ellipsoid/1000;

              Serial.write('P');
              doubleToBytes.value = nav.lat;
              Serial.write(doubleToBytes.bytesArr, sizeof(double));
              doubleToBytes.value = nav.lng;
              Serial.write(doubleToBytes.bytesArr, sizeof(double));
              doubleToBytes.value = nav.alt;
              Serial.write(doubleToBytes.bytesArr, sizeof(double));
              uint64ToBytes.value = nav.iTOW;
              Serial.write(uint64ToBytes.bytesArr, sizeof(uint64_t));
              Serial.write('\n');
            }
            break;
          
          case 58: // SOL string received
            if (CK_A == GPS_info_buffer[57] && CK_B == GPS_info_buffer[58]) { // confirm received and calculated checksums are the same
              fixok = (( GPS_info_buffer[11 + 5] & B00000001) && ( GPS_info_buffer[10 + 5]) == 0x03); // fix status flag = 1 AND a 3Dfix
            }
          break;
        }
        // ubxPrint();
        message_started = false; // ready for the new message
      } // end final character received
    } // end the message is already started and I got a new character
  } // end If GPS data is available

  return NOGPS;
} // end loop

#endif