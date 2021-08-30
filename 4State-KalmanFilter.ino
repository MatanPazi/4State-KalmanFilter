#include <Wire.h>
#include <Servo.h>
//Servo myservo;  // create servo object to control a servo
#define NUM_OF_ITERATIONS 1000 
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
int pos = 0;
long counter = 0;
long GyroErrors = 0;
bool IsFirstRun = true;
long loopTime = 5000;  // Microseconds
unsigned long timer = 0;
int WhichError = 0;

void setup() {
  Serial.begin(115200);
  timer = micros();
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  /*
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  */
  // Call this function if you need to get the IMU error values for your module
  GyroErrorX = calculate_IMU_error(WhichError);
  WhichError = 1;
  GyroErrorY = calculate_IMU_error(WhichError);
  WhichError = 2;
  GyroErrorZ = calculate_IMU_error(WhichError);
  delay(10000);
}
void loop() {
  // === Read acceleromter data === //
  timeSync(loopTime);
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 - 0.02; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 - 0.02; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI); // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI); // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX; // GyroErrorX ~(-0.56)
  GyroY = GyroY - GyroErrorY; // GyroErrorY ~(2)
  GyroZ = GyroZ - GyroErrorZ; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  // Initializing
  if (IsFirstRun)
  {
    gyroAngleX = accAngleX;
    gyroAngleY = accAngleY;
  }
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  IsFirstRun = false;
  sendToPC(&pitch, &roll, &yaw);

  // Print the values on the serial monitor
//  Serial.println(pitch);
//  Serial.print("/");
//  Serial.print(roll);
//  Serial.print("/");
//  Serial.println(yaw);
//  Serial.print("/");
  // Command servo to go according to roll angle
  //pos = 90 - pitch;
  //myservo.write(pos);
//  Check Servo angle command relationship to actual angle
//  counter = counter + 1;
//  if (counter == 1) 
//  {
//    myservo.write(130);
//    delay(10);
//  }
//  else if (counter == 1000)
//  {
//    myservo.write(110);    
//    delay(10);
//  }
//  else if (counter == 2000)
//  {
//    myservo.write(90);    
//    delay(10);
//  }
//  else if (counter == 3000)
//  {
//    myservo.write(70);    
//    delay(10);
//  }
//  else if (counter == 4000)
//  {
//    myservo.write(50);   
//    delay(10); 
//  }
//  else
//  {
//    sendToPC(&pitch, &roll, &yaw);
//  }
}
float calculate_IMU_error(int WhichError) {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  float result;
  float AccErrorX = 0, AccErrorY = 0, AccErrorZ = 0;
  float GyroErrorX = 0, GyroErrorY = 0, GyroErrorZ = 0;
  float AccX, AccY, AccZ;
  float GyroX, GyroY, GyroZ;
//  while (c < NUM_OF_ITERATIONS) {
//    Wire.beginTransmission(MPU);
//    Wire.write(0x3B);
//    Wire.endTransmission(false);
//    Wire.requestFrom(MPU, 6, true);
//    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
//    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
//    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
//    // Sum all readings
//    AccErrorX = AccErrorX + AccX; //((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
//    AccErrorY = AccErrorY + AccY; //((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
//    AccErrorZ = AccErrorZ + AccZ;
//    c++;
//  }
//  //Divide the sum by NUM_OF_ITERATIONS to get the error value
//  AccErrorX = AccErrorX / NUM_OF_ITERATIONS;
//  AccErrorY = AccErrorY / NUM_OF_ITERATIONS;
//  AccErrorZ = 1 - AccErrorZ / NUM_OF_ITERATIONS;
  c = 0;
  // Read gyro values NUM_OF_ITERATIONS times
  while (c < NUM_OF_ITERATIONS) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by NUM_OF_ITERATIONS to get the error value
  GyroErrorX = GyroErrorX / NUM_OF_ITERATIONS;
  GyroErrorY = GyroErrorY / NUM_OF_ITERATIONS;
  GyroErrorZ = GyroErrorZ / NUM_OF_ITERATIONS;
  // Print the error values on the Serial Monitor
//  Serial.print("AccErrorX: ");
//  Serial.println(AccErrorX);
//  Serial.print("AccErrorY: ");
//  Serial.println(AccErrorY);
//  Serial.print("AccErrorZ: ");
//  Serial.println(AccErrorZ);
//  Serial.print("GyroErrorX: ");
//  Serial.println(GyroErrorX);
//  Serial.print("GyroErrorY: ");
//  Serial.println(GyroErrorY);
//  Serial.print("GyroErrorZ: ");
//  Serial.println(GyroErrorZ);
  if (WhichError == 0)
  {
    result = GyroErrorX;
  }
  else if (WhichError == 1)
  {
    result = GyroErrorY;
  }
  else
  {
    result = GyroErrorZ;
  }
  return result;
}

void timeSync(unsigned long deltaT)
{
  unsigned long currTime = micros();
  long timeToDelay = deltaT - (currTime - timer);
  if (timeToDelay > 5000)
  {
    delay(timeToDelay / 1000);
    delayMicroseconds(timeToDelay % 1000);
  }
  else if (timeToDelay > 0)
  {
    delayMicroseconds(timeToDelay);
  }
  else
  {
      // timeToDelay is negative so we start immediately
  }
  timer = currTime + timeToDelay;
}

void sendToPC(float* data1, float* data2, float* data3)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte buf[12] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3]};
  Serial1.write(buf, 12);
}
