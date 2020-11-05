#include "U8glib.h"
U8GLIB_SSD1306_128X64 u8g(A5, A3);    // OLED display: HW SPI CS = 10, A0/DC = 9 (Hardware Pins are  SCK/D0 = 13 and MOSI/D1 = 11) + RST to Arduino D8
#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH // Comment out to restrict roll to Â±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

float roll;
float pitch;
int centerX = 64;
int centerY = 32;
float posX = centerX;
float posY = centerY;
int targetX;
int targetY;
int success_counter = 0;
long start_time;
boolean basic;
int sec = 60;

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////



void setup() {

  pinMode(A4, OUTPUT);                            // RST pin for OLED display
  digitalWrite(A4, LOW);
  delay(100);
  digitalWrite(A4, HIGH);

  pinMode(6, OUTPUT);             // LED
  pinMode(9, OUTPUT);             // buzzer
//  tone(9, 500);
//  delay(100);
//  tone(9, 1000);
//  delay(100);
//  noTone(9);

  pinMode(7, INPUT_PULLUP);       // game or basic
  basic = digitalRead(7);

  randomSeed(analogRead(0));
  setRandom();

  Serial.begin(115200);
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to Â±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to Â±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

//  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -Ï€ to Ï€ (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll  = atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

//  if (!basic) drawIntroGame();
//  else 


  timer = micros();
  start_time = millis();
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

void loop() {

  getIMU();      // this takes about 2msec
//  if (!basic) {
    drawIMUgame();
//  }
//  else {
//    drawIMUbasic();
//    int distance = sq(roll) + sq(pitch);
//    tone(9, distance + 100);
//
//  }

  Serial.print(roll);
  Serial.print("\t");
  Serial.println(pitch);


}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

void getIMU() {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -Ï€ to Ï€ (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll  = atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");

  Serial.print(roll); Serial.print("\t");
  Serial.print(gyroXangle); Serial.print("\t");
  Serial.print(compAngleX); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("\t");

  Serial.print(pitch); Serial.print("\t");
  Serial.print(gyroYangle); Serial.print("\t");
  Serial.print(compAngleY); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");

  Serial.print("\t");

  double temperature = (double)tempRaw / 340.0 + 36.53;
  Serial.print(temperature); Serial.print("\t");
  Serial.println();
#endif
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
//
void drawIMUgame() {
  u8g.firstPage();
  do {
  u8g.setFont(u8g_font_7x14B);
    u8g.setPrintPos(2, 12);
    u8g.print("p:");
    u8g.print(pitch, 1);
    u8g.print("\xB0");
    u8g.setPrintPos(2, 25);
    u8g.print("r:");
    u8g.print(roll, 1);
    u8g.print("\xB0");
    u8g.drawFrame(66, 0, 62, 64);
//        u8g.drawCircle(96,32,30);
//        u8g.drawHLine(66, centerY, 62);
//        u8g.drawVLine(centerX, 1, 62);
    u8g.drawFrame(targetX - 2, targetY - 2, 6, 6);
    posX -= roll / 10.0;
    posY += pitch / 10.0;
    posX = constrain(posX, 67, 126);
    posY = constrain(posY, 1, 62);
    u8g.drawDisc(posX, posY, 4);
    u8g.setPrintPos(2, 62);
    u8g.print(abs(sec - (millis() - start_time) / 1000.0), 0);
    u8g.print("s");

    if (posX > (targetX - 2) && posX < (targetX + 2) && posY > (targetY - 2) && posY < (targetY + 2)) {
      success_counter++;
      digitalWrite(6, HIGH);
      setRandom();
      tone(9, 1000, 100);
    }
    else {
      digitalWrite(6, LOW);
    }
    u8g.setPrintPos(2, 48);
    u8g.print("score:");
    u8g.print(success_counter);
  } while ( u8g.nextPage() );
//  if (posX == 67 || posX == 126 || posY == 1 || posY == 62) {                         // HIT BORDER = GAME OVER
//    tone(9, 100, 1000);
//    while (1) {}
//  }
//  if ((millis() - start_time) / 1000.0 >= sec) {                                        // many SEC = GAME OVER
//    for (int i=0;i<30;i++) {
//    tone(9, 2000);
//    digitalWrite(6, LOW);
//    delay(20);
//    noTone(9);
//    digitalWrite(6, HIGH);
//    delay(20);
//    }
//    while (1) {}
//  }
}

void setRandom() {
  targetX = random(68, 125);
  targetY = random(2, 61);
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

//void drawIMUbasic() {
//  u8g.firstPage();
//  do {
////    u8g.setFont(u8g_font_7x14B);
//    u8g.setPrintPos(0, 6);
//    u8g.print("pitch:");
//    u8g.setPrintPos(2, 26);
//    u8g.print(pitch, 1);
//    u8g.print("\xB0");
//    u8g.setPrintPos(2, 45);
//    u8g.print("roll:");
//    u8g.setPrintPos(2, 59);
//    u8g.print(roll, 1);
//    u8g.print("\xB0");
//    //u8g.drawFrame(66, 0, 62, 64);
//    //u8g.drawCircle(96,32,30);
//    u8g.drawHLine(posX, posY, 126);
//    u8g.drawVLine(posX, posY, 126);
////    u8g.setColorIndex(0);
////    u8g.drawDisc(centerX, centerY, 4);
////    u8g.setColorIndex(1);
//    u8g.drawCircle(centerX, centerY, 8);
////        int posX = 128 + 30.0*roll/90.0;
////        int posY = 64 + 30.0*pitch/90.0;
//        int posX = 96 + 30.0*sin(roll*3.14/180.0);
//        int posY = 32 - 30.0*sin(pitch*3.14/180.0);
//    posX = centerX - roll / 2.0;
//    posY = centerY + pitch / 2.0;
//    posX = constrain(posX, 0, 128);
//    posY = constrain(posY, 0, 64);
//    u8g.drawDisc(posX, posY, 2);
//  } while ( u8g.nextPage() );
//}





////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

//void drawIntroGame() {
//  u8g.firstPage();
//  do {
//    u8g.setFont(u8g_font_fub14r);
//    u8g.setPrintPos(2, 30);
//    u8g.print("watchX");
//    u8g.setPrintPos(12, 60);
//    u8g.print("Gyro Game");
//  } while ( u8g.nextPage() );
//}

void drawIntroBasic() {
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_fub14r);
    u8g.setPrintPos(2, 30);
    u8g.print("watchX");
    u8g.setPrintPos(2, 60);
    u8g.print("Gyro Game");
  } while ( u8g.nextPage() );
}
