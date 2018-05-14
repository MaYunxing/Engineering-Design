
/*
 * Acceleration and Pressure Detection
 *
 * modified 4/30/2018
 * by Ma Yunxing
 */
#include <SPI.h> 
#include <Wire.h>
#include "Adafruit_LIS3DH.h"
//#include <Adafruit_LIS3DH.cpp>
#include "Adafruit_Sensor.h"    // libraries provided by producer
#include <Arduino.h>
#include <SD.h>
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

 float initialpressure;
#define LIS3DH_CLK 13

#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
#define LIS3DH_CS 10


File myFile;
File myFile1;



void setup()
{
  Serial.begin(9600);
  Wire.begin();
  while (!Serial){
    ;
  }
  const int chipSelect = 10;
  Serial.print("Initializing SD card");
  pinMode(chipSelect,OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present.");
    return;
  }
 Serial.println("Card Initialized.");



  Serial.println("LIS3DH test!"); //check the availability of the sensor
  
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH found!");
  
  lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
  
  Serial.print("Range = "); Serial.print(2 << lis.getRange());  
  Serial.println("G");


  myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening pressure.txt");
  }
  //myFile1 = SD.open("accelerationdata.txt", FILE_WRITE);
  //if (myFile1) {
    //Serial.print("Writing to test.txt...");
    //myFile1.println("testing 1, 2, 3.");
    // close the file:
    //myFile1.close();
    //Serial.println("done.");
  //} else {
    // if the file didn't open, print an error:
    //Serial.println("error opening Acceleration.txt");
  //}
  
  
}
  
void loop() {
  getacceleration();
    getpressure();
  delay(50); 
  //const int chipSelect = 10;
  //Serial.print("Initializing SD card");
  //String dataString = " ";
  //pinMode(chipSelect,OUTPUT);
  //if (!SD.begin(chipSelect)) {
    //Serial.println("Card failed, or not present.");
    //return;
  //}
 //Serial.println("Card Initialized.");

}
void getpressure()
{

  // float pressure = ( analogRead(A2) - SensorOffset)/100.0;
  int pressurePin = A0;

  initialpressure = analogRead(pressurePin);
  float pressure;
  pressure = (((((initialpressure/(float)1023)+3.47)/5)+0.095))/0.009;
  Serial.print("pressure = ");
  Serial.print(pressure,4);
  Serial.print(" kPa ");

  File dataFile = SD.open("test.txt", FILE_WRITE);
  if (dataFile) {
  dataFile.println(pressure);
  dataFile.close();
 }
else{
  Serial.println("Error opening PressureData.txt");
 }

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



void getacceleration()
{
  //Adafruit_LIS3DH lis = Adafruit_LIS3DH()
 lis.read();      // get X Y and Z data at once
  // Then print out the raw data
  Serial.print("X:  "); Serial.print(lis.x); 
  Serial.print("  \tY:  "); Serial.print(lis.y); 
  Serial.print("  \tZ:  "); Serial.print(lis.z); 

  
  sensors_event_t event; 
  lis.getEvent(&event);
  

  Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
  Serial.print(" \tY: "); Serial.print(event.acceleration.y); 
  Serial.print(" \tZ: "); Serial.print(event.acceleration.z); 
  Serial.println(" m/s^2 ");

  Serial.println();
 
  

 File dataFile = SD.open("test.txt",FILE_WRITE);
 if (dataFile) {
  dataFile.println(event.acceleration.z);
  dataFile.close();
 }
 else{
  Serial.println("Error opening AccelerationData.txt");
 }
}
