#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <SoftwareSerial.h>
#define RESTRICT_PITCH // Comment out to restrict roll to 짹90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

float timePassed = 0;

// TODO: Make calibration routine

int fingerPosition[4];

String sendMsg;

String rawGyroValue[3];
String gyroPM[3];
String rawFingerValue[4];


String finalData[7];

SoftwareSerial BTport(2, 3);

int i;

void setup() {
  BTport.begin(9600);
  Serial.begin(115200);
}
  
void loop() {
  // put your main code here, to run repeatedly:
  fingerPosition[0] = analogRead(A0);
  fingerPosition[1] = analogRead(A1);
  fingerPosition[2] = analogRead(A2);
  fingerPosition[3] = analogRead(A3);
  sendMsg = "";
  for(i = 0; i < 7; i++)
  {
    finalData[i] = "";
  }
  
  GetGyrosensorValue();
  gyroX = (((gyroX / 1200) + 1) / 1.1 * -1 +0.8);
  gyroY = gyroY/600/1.1;
  gyroZ = gyroZ/600/-1;

  Serial.print(gyroX);
  Serial.print("  :  ");
  Serial.print(gyroY);
  Serial.print("  :  ");
  Serial.print(gyroZ);
  Serial.print("  :  ");
  
  rawGyroValue[0] = (String)(int)abs(gyroX);
  rawGyroValue[1] = (String)(int)abs(gyroY);
  rawGyroValue[2] = (String)(int)abs(gyroZ);

  if(gyroX >= 0)gyroPM[0] = ">";
  if(gyroX < 0)gyroPM[0] = "<";
  if(gyroY >= 0)gyroPM[1] = ">";
  if(gyroY < 0)gyroPM[1] = "<";
  if(gyroZ >= 0)gyroPM[2] = ">";
  if(gyroZ < 0)gyroPM[2] = "<";
  
  for(i = 0; i < 3; i++)
  {
    finalData[i] += gyroPM[i];
    if(strlen(rawGyroValue[i].c_str()) < 3)
    {
      for(int j = 0; j < 3 - strlen(rawGyroValue[i].c_str()); j++)
      {
        finalData[i] += "0";
      }
    }
    finalData[i] += rawGyroValue[i];
  }
  for(i = 3; i < 7; i++)
  {
    finalData[i] = String(fingerPosition[i-3]);
  }
  for(i = 0; i < 7; i++)
  {
    sendMsg += finalData[i];
  }
  sendMsg += "/";
  Serial.println(sendMsg);
  BTport.print(sendMsg);
  
 
  /*

  
  for(i = 0; i < 4; i++)
  {
    rawFingerValue[i] = (String)fingerPosition[i];
  }

  

  for(i = 3; i < 7; i++)
  {
    if(strlen(rawFingerValue[i].c_str()) < 4)
    {
      for(int j = 0; j < 5 - strlen(rawFingerValue[i].c_str()); j++)
      {
        fixedData[i] += "0";
      }
    }
    fixedData[i] += rawFingerValue[i];
  }

 

  BTport.println(sendMsg);
  Serial.println(sendMsg);*/
/*Serial.print("\t");
Serial.print(gyroY);Serial.print("\t");
Serial.print(gyroZ);Serial.print("\t");

while(gyroY>40){
  Serial.println("Drag Down");
  timePassed +=1;
  if (timePassed>100){
    break;
  }
  GetGyrosensorValue();
  while(gyroZ>40&&gyroY<-40){
    Serial.println("Drag UpLeft");
    timePassed += 1;
    if(timePassed > 150){
      timePassed = 0;
      break;
    
  }
  Serial.print(timePassed);
  Serial.print("\t");
  delay(10);
}
  delay(10);
  */
}

void GetGyrosensorValue(){
  // put your setup code here, to run once:      Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to 짹250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to 짹2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -� to � (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -� to � (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
 /* double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;*/
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
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

  /* Print Data 
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

  Serial.print(roll); Serial.print("\t");
  Serial.print(gyroXangle); Serial.print("\t");
  Serial.print(compAngleX); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("\t");

  Serial.print(pitch); Serial.print("\t");
  Serial.print(gyroYangle); Serial.print("\t");
  Serial.print(compAngleY); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");

#if 0 // Set to 1 to print the temperature
  Serial.print("\t");

  double temperature = (double)tempRaw / 340.0 + 36.53;
  Serial.print(temperature); Serial.print("\t");
#endif

  Serial.print("\r\n");*/
  delay(2);
}
