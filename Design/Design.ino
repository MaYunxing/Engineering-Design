/*
 * Acceleration and Pressure Detection
 *
 * modified 4/30/2018
 * by Ma Yunxing
 */
#include <SPI.h>
#include <Wire.h>
#include <SD.h>

const int chipSelect = 10;
const float SensorOffset = 102.0;

float sine_angle_z;
float angle_z_degrees;
float pressure;



#define ACC (0xA7>>1)
#define A_TO_READ (6)

void loop() {
  getacceleration();
  getpressure();

}
void getpressure()
{
  float pressure = ( analogRead(A2) - SensorOffset)/100.0;
  asd
  Serial.print("pressure = ");
  Serial.print(pressure,4);
  Serial.print(" kPa ");
  delay(100);
  File dataFile = SD.open("PressureData.txt", FILE_WRITE);
  if (dataFile) {
  dataFile.println(pressure);
  dataFile.close();
 }
 else{
  Serial.println("Error opening PressureData.txt");
 }

}

void initAcc() {
  writeTo(ACC, 0x2D,1<<3);
  writeTo(ACC, 0x31,0x0B);
  writeTo(ACC, 0x2C,0x09);

}
void getAccelerometerData(int* result) {
  int regAddress = 0x32;
  byte buff[A_TO_READ];
  readFrom(ACC,regAddress,A_TO_READ, buff);
  result[0] = (((int)buff[1]) << 8) | buff[0];
  result[1] = (((int)buff[3] )<< 8) | buff[2];
  result[2] = (((int)buff[5]) << 8) | buff[4];
}

void writeTo(int DEVICE, byte address, byte val){
  Wire.beginTransmission(DEVICE);
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission();
}

void readFrom(int DEVICE, byte address, int num, byte buff[]) {
  Wire.beginTransmission(DEVICE);
  Wire.write(address);
  Wire.endTransmission();
  Wire.beginTransmission(DEVICE);
  Wire.requestFrom(DEVICE,num);
  int i = 0;
  while(Wire.available()){
    buff[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();
}
void setup()
{
  Serial.begin(9600);
  Wire.begin();
  initAcc();// put your setup code here, to run once:

  Serial.print("Initializing SD card");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present.");
    return;
  }
  Serial.println("Card Initialized.");

}
void getacceleration()
{int x,y,z;// put your main code here, to run repeatedly:
  int acc[3];

  getAccelerometerData(acc);
  z = acc[2];

  Serial.print("sine_angle_z = ");
  Serial.print(sine_angle_z);
  Serial.print("  ");
  sine_angle_z =((z+10.00)/240.00);

  Serial.print (" sine_angle_z =");
  Serial.print (sine_angle_z);
  Serial.print ("   ");

  if (sine_angle_z >1) {
    sine_angle_z = -1;

  }
  else if (sine_angle_z<-1) {
    sine_angle_z = -1;
  }

 angle_z_degrees = asin(sine_angle_z) * RAD_TO_DEG;
 Serial.print( "angle_z_degrees = ");
 Serial.println(angle_z_degrees);

 delay(100);

 File dataFile = SD.open("AccelerationData.txt",FILE_WRITE);
 if (dataFile) {
  dataFile.println(angle_z_degrees);
  dataFile.close();
 }
 else{
  Serial.println("Error opening AccelerationData.txt");
 }
}
