/*
   Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial
   by Dejan, https://howtomechatronics.com
*/
#include <Wire.h>
#define NUM_OF_ITERATIONS 500

float AccX, AccY, AccZ;

float AccXVar, AccYVar, AccZVar;
float AccXAngVar, AccYAngVar;
float AccXAng, AccYAng;
float AccMeanX, AccMeanY, AccMeanZ;
float AccAngMeanX, AccAngMeanY;
float GyroX, GyroY, GyroZ;
float GyroXVar, GyroYVar, GyroZVar;
const int MPU = 0x68; // MPU6050 I2C address

void setup() {
  Serial.begin(115200);
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
}
void loop() {


  float AccXSum = 0, AccYSum = 0, AccZSum = 0;
  float AccXSumSquare = 0, AccYSumSquare = 0, AccZSumSquare = 0;
  float AccXAngSumSquare = 0, AccYAngSumSquare = 0;
  float AccXAngSum = 0, AccYAngSum = 0;
  float GyroXSumSquare = 0, GyroYSumSquare = 0, GyroZSumSquare = 0;
  float GyroMeanX = 0, GyroMeanY = 0, GyroMeanZ = 0;
  int c = 0;

  while (c < NUM_OF_ITERATIONS) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Calculate sum of readings for mean calculations
    AccXSum = AccXSum + AccX;
    AccYSum = AccYSum + AccY;
    AccZSum = AccZSum + AccZ;    
    // Angle Calculations
    AccXAng = ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccYAng = ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    // Calculate the sum of angles for mean value
    AccXAngSum = AccXAngSum + AccXAng;
    AccYAngSum = AccYAngSum + AccYAng;
    c++;
  }
  
  //Divide the sum by NUM_OF_ITERATIONS to get the Mean value
  AccMeanX = AccXSum / NUM_OF_ITERATIONS;
  AccMeanY = AccYSum / NUM_OF_ITERATIONS;
  AccMeanZ = AccZSum / NUM_OF_ITERATIONS;
  AccAngMeanX = AccXAngSum / NUM_OF_ITERATIONS;
  AccAngMeanY = AccYAngSum / NUM_OF_ITERATIONS;
  
  c = 0;
  while (c < NUM_OF_ITERATIONS) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
 
    // Angle Calculations
    AccXAng = ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccYAng = ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    
    // Calculate the sum squared of reading minus mean value
    AccXSumSquare = AccXSumSquare + sq(AccX - AccMeanX);
    AccYSumSquare = AccYSumSquare + sq(AccY - AccMeanY);
    AccZSumSquare = AccZSumSquare + sq(AccZ - AccMeanZ);

    // Calculate the sum squared of angles minus mean value
    AccXAngSumSquare = AccXAngSumSquare + sq(AccXAng - AccAngMeanX);
    AccYAngSumSquare = AccYAngSumSquare + sq(AccYAng - AccAngMeanY);
    c++;
  }
  
  AccXVar = AccXSumSquare / NUM_OF_ITERATIONS;
  AccYVar = AccYSumSquare / NUM_OF_ITERATIONS;
  AccZVar = AccZSumSquare / NUM_OF_ITERATIONS;
  AccXAngVar = AccXAngSumSquare / NUM_OF_ITERATIONS;
  AccYAngVar = AccYAngSumSquare / NUM_OF_ITERATIONS;
  
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
    GyroMeanX = GyroMeanX + (GyroX / 131.0);
    GyroMeanY = GyroMeanY + (GyroY / 131.0);
    GyroMeanZ = GyroMeanZ + (GyroZ / 131.0);
    c++;
  }
  
  //Divide the sum by NUM_OF_ITERATIONS to get the Mean value
  GyroMeanX = GyroMeanX / NUM_OF_ITERATIONS;
  GyroMeanY = GyroMeanY / NUM_OF_ITERATIONS;
  GyroMeanZ = GyroMeanZ / NUM_OF_ITERATIONS;

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
    
    // Calculate the sum squared of readings minus mean value
    GyroXSumSquare = GyroXSumSquare + sq(GyroX / 131.0 - GyroMeanX);
    GyroYSumSquare = GyroYSumSquare + sq(GyroY / 131.0 - GyroMeanY);
    GyroZSumSquare = GyroZSumSquare + sq(GyroZ / 131.0 - GyroMeanZ);
    c++;
  }  

  GyroXVar = GyroXSumSquare / NUM_OF_ITERATIONS;
  GyroYVar = GyroYSumSquare / NUM_OF_ITERATIONS;
  GyroZVar = GyroZSumSquare / NUM_OF_ITERATIONS;
  
  // Print the values on the Serial Monitor
  Serial.print("AccMeanX: ");
  Serial.println(AccMeanX, 4);
  Serial.print("AccMeanY: ");
  Serial.println(AccMeanY, 4);
  Serial.print("AccMeanZ: ");
  Serial.println(AccMeanZ, 4);
  
  Serial.print("AccAngMeanX: ");
  Serial.println(AccAngMeanX, 4);
  Serial.print("AccAngMeanY: ");
  Serial.println(AccAngMeanY, 4);
  
  Serial.print("AccXVar: ");
  Serial.println(AccXVar, 6);
  Serial.print("AccYVar: ");
  Serial.println(AccYVar, 6);
  Serial.print("AccZVar: ");
  Serial.println(AccZVar, 6);

  Serial.print("AccXAngVar: ");
  Serial.println(AccXAngVar, 6);
  Serial.print("AccYAngVar: ");
  Serial.println(AccYAngVar, 6);

  Serial.print("GyroMeanX: ");
  Serial.println(GyroMeanX, 4);
  Serial.print("GyroMeanY: ");
  Serial.println(GyroMeanY, 4);
  Serial.print("GyroMeanZ: ");
  Serial.println(GyroMeanZ, 4);

  Serial.print("GyroXVar: ");
  Serial.println(GyroXVar, 6);
  Serial.print("GyroYVar: ");
  Serial.println(GyroYVar, 6);
  Serial.print("GyroZVar: ");
  Serial.println(GyroZVar, 6);

  delay(10000);
  Serial.print("\n \n");

}
