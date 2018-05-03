

/*
 * Acceleration and Pressure Detection
 *
 * modified 4/30/2018
 * by Ma Yunxing
 */
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <SD.h>

const int chipSelect = 10;
const float SensorOffset = 102.0;

float sine_angle_z;
float angle_z_degrees;
float pressure;

#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
#define LIS3DH_CS 10

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
  #ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  Serial.begin(9600);
  Serial.println("LIS3DH test!");
  
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH found!");
  
  lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
  
  Serial.print("Range = "); Serial.print(2 << lis.getRange());  
  Serial.println("G");
}


}
void getacceleration()
{
 lis.read();      // get X Y and Z data at once
  // Then print out the raw data
  Serial.print("X:  "); Serial.print(lis.x); 
  Serial.print("  \tY:  "); Serial.print(lis.y); 
  Serial.print("  \tZ:  "); Serial.print(lis.z); 

  /* Or....get a new sensor event, normalized */ 
  sensors_event_t event; 
  lis.getEvent(&event);
  
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
  Serial.print(" \tY: "); Serial.print(event.acceleration.y); 
  Serial.print(" \tZ: "); Serial.print(event.acceleration.z); 
  Serial.println(" m/s^2 ");

  Serial.println();
 
  delay(200); 

 File dataFile = SD.open("AccelerationData.txt",FILE_WRITE);
 if (dataFile) {
  dataFile.println(angle_z_degrees);
  dataFile.close();
 }
 else{
  Serial.println("Error opening AccelerationData.txt");
 }
}
