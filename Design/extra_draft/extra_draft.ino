

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

float pressure;

#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
#if defined(ARDUINO_ARCH_SAMD)
#define LIS3DH_REG_STATUS1       0x07
#define LIS3DH_REG_OUTADC1_L     0x08
#define LIS3DH_REG_OUTADC1_H     0x09
#define LIS3DH_REG_OUTADC2_L     0x0A
#define LIS3DH_REG_OUTADC2_H     0x0B
#define LIS3DH_REG_OUTADC3_L     0x0C
#define LIS3DH_REG_OUTADC3_H     0x0D
#define LIS3DH_REG_INTCOUNT      0x0E
#define LIS3DH_REG_WHOAMI        0x0F
#define LIS3DH_REG_TEMPCFG       0x1F
#define LIS3DH_REG_CTRL1         0x20
#define LIS3DH_REG_CTRL2         0x21
#define LIS3DH_REG_CTRL3         0x22
#define LIS3DH_REG_CTRL4         0x23
#define LIS3DH_REG_CTRL5         0x24
#define LIS3DH_REG_CTRL6         0x25
#define LIS3DH_REG_REFERENCE     0x26
#define LIS3DH_REG_STATUS2       0x27
#define LIS3DH_REG_OUT_X_L       0x28
#define LIS3DH_REG_OUT_X_H       0x29
#define LIS3DH_REG_OUT_Y_L       0x2A
#define LIS3DH_REG_OUT_Y_H       0x2B
#define LIS3DH_REG_OUT_Z_L       0x2C
#define LIS3DH_REG_OUT_Z_H       0x2D
#define LIS3DH_REG_FIFOCTRL      0x2E
#define LIS3DH_REG_FIFOSRC       0x2F
#define LIS3DH_REG_INT1CFG       0x30
#define LIS3DH_REG_INT1SRC       0x31
#define LIS3DH_REG_INT1THS       0x32
#define LIS3DH_REG_INT1DUR       0x33
#define LIS3DH_REG_CLICKCFG      0x38
#define LIS3DH_REG_CLICKSRC      0x39
#define LIS3DH_REG_CLICKTHS      0x3A
#define LIS3DH_REG_TIMELIMIT     0x3B
#define LIS3DH_REG_TIMELATENCY   0x3C
#define LIS3DH_REG_TIMEWINDOW    0x3D
#define LIS3DH_REG_ACTTHS        0x3E
#define LIS3DH_REG_ACTDUR        0x3F

typedef enum
{
  LIS3DH_RANGE_16_G         = 0b11,   // +/- 16g
  LIS3DH_RANGE_8_G           = 0b10,   // +/- 8g
  LIS3DH_RANGE_4_G           = 0b01,   // +/- 4g
  LIS3DH_RANGE_2_G           = 0b00    // +/- 2g (default value)
} lis3dh_range_t;

typedef enum
{
  LIS3DH_AXIS_X         = 0x0,
  LIS3DH_AXIS_Y         = 0x1,
  LIS3DH_AXIS_Z         = 0x2,
} lis3dh_axis_t;


/* Used with register 0x2A (LIS3DH_REG_CTRL_REG1) to set bandwidth */
typedef enum
{
  LIS3DH_DATARATE_400_HZ     = 0b0111, //  400Hz 
  LIS3DH_DATARATE_200_HZ     = 0b0110, //  200Hz
  LIS3DH_DATARATE_100_HZ     = 0b0101, //  100Hz
  LIS3DH_DATARATE_50_HZ      = 0b0100, //   50Hz
  LIS3DH_DATARATE_25_HZ      = 0b0011, //   25Hz
  LIS3DH_DATARATE_10_HZ      = 0b0010, // 10 Hz
  LIS3DH_DATARATE_1_HZ       = 0b0001, // 1 Hz
  LIS3DH_DATARATE_POWERDOWN  = 0,
  LIS3DH_DATARATE_LOWPOWER_1K6HZ  = 0b1000,
  LIS3DH_DATARATE_LOWPOWER_5KHZ  =  0b1001,

} lis3dh_dataRate_t;

class Adafruit_LIS3DH : public Adafruit_Sensor {
 public:
  Adafruit_LIS3DH(void);
  Adafruit_LIS3DH(TwoWire *Wi);
  Adafruit_LIS3DH(int8_t cspin);
  Adafruit_LIS3DH(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin);
  
  bool       begin(uint8_t addr = LIS3DH_DEFAULT_ADDRESS);
 
  void read();
  int16_t readADC(uint8_t a);

  void setRange(lis3dh_range_t range);
  lis3dh_range_t getRange(void);

  void setDataRate(lis3dh_dataRate_t dataRate);
  lis3dh_dataRate_t getDataRate(void);

  bool getEvent(sensors_event_t *event);
  void getSensor(sensor_t *sensor);

  uint8_t getOrientation(void);

  void setClick(uint8_t c, uint8_t clickthresh, uint8_t timelimit = 10, uint8_t timelatency = 20, uint8_t timewindow = 255);
  uint8_t getClick(void);

  int16_t x, y, z;
  float x_g, y_g, z_g;

 private:
  TwoWire *I2Cinterface;

  uint8_t readRegister8(uint8_t reg);
  void writeRegister8(uint8_t reg, uint8_t value);
  uint8_t spixfer(uint8_t x = 0xFF);


  int32_t _sensorID;
  int8_t  _i2caddr;

  // SPI
  int8_t _cs, _mosi, _miso, _sck;
};




   #define Serial SerialUSB
#endif

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  

  Serial.print("Initializing SD card");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present.");
    return;
  }
  Serial.println("Card Initialized.");
  #ifndef ESP8266
   while (!Serial);   
  #endif

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


void loop() {
  getacceleration();
  getpressure();
  delay(50); 

}
void getpressure()
{
  
  // float pressure = ( analogRead(A2) - SensorOffset)/100.0;
  //
  initialpressure = analogRead(pressurePin);

  pressure = ((float)initialpressure/(float)1023+0.095)/0.009;
  Serial.print("pressure = ");
  Serial.print(pressure,4);
  Serial.print(" kPa ");

  File dataFile = SD.open("PressureData.txt", FILE_WRITE);
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
 
  

 File dataFile = SD.open("AccelerationData.txt",FILE_WRITE);
 if (dataFile) {
  dataFile.println(event.acceleration.z);
  dataFile.close();
 }
 else{
  Serial.println("Error opening AccelerationData.txt");
 }
}
