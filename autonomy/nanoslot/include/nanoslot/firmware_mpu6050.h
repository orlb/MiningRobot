/*
 Talk to the MPU-6050 inertial measurement unit (IMU).
*/
#ifndef NANOSLOT_FIRMWARE_MPU_6050_H
#define NANOSLOT_FIRMWARE_MPU_6050_H
/*

 Jumpers for Arduno Uno or Nano:
 Vcc Vin
 GND GND
 A4 SDA
 A5 SCL
 
 You want the "register map and descriptions" from here:
    https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf
 Unless you're building a board the datasheet is useless!

 Hardware: the sub-$1 "ITG/MPU" 6050 knockoff boards on AliExpress.
 
 The 6050 part has no compass hardware (reads as zeros)
  - Accelerometer is pretty tight
  - Gyro has a small constant offset, a few hundred units
  - Onboard filtering makes data smoother but slower to update

 Using cheap AliExpress nano knockoff too.
 On Arudino 1.8.5 IDE, 
  Board: Arduino Nano
  Processor: ATmega328 (New bootloader)

 On Arduino 1.6.5 IDE, I had to flash it as:
  Board: Arduino mini (note: not Nano, due to new bootloader?)
  Processor: ATmega328

  Port: /dev/ttyUSB0
  Programmer: AVRISP mkII
 
 
 This began life as example code from:
 http://playground.arduino.cc/Main/MPU-9150
 MPU-9150 Accelerometer + Gyro + Compass + Temperature
 */
// -----------------------------
//
// By arduino.cc user "frtrobotik" (Tobias Hübner)
//
//
// July 2013
//      first version
//
// Open Source / Public Domain
//
// Documentation:
// - The InvenSense documents:
//   - "MPU-9150 Product Specification Revision 4.0",
//     http://www.invensense.com/mems/gyro/documents/PS-MPU-9150A.pdf
//   - "MPU-9150 Register Map and Descriptions Revision 4.0",
//      http://www.invensense.com/mems/gyro/documents/RM-MPU-9150A-00v4_2.pdf

// The accuracy is 16-bits.
//
// Some parts are copied by the MPU-6050 Playground page.
// playground.arduino.cc/Main/MPU-6050
// There are more Registervalues. Here are only the most
// necessary ones to get started with this sensor.

#include <Wire.h>

// Register names according to the datasheet.
// According to the InvenSense document
// "MPU-9150 Register Map and Descriptions Revision 4.0",

#define MPU_SELF_TEST_X        0x0D   // R/W
#define MPU_SELF_TEST_Y        0x0E   // R/W
#define MPU_SELF_TEST_Z        0x0F   // R/W
#define MPU_SELF_TEST_A        0x10   // R/W
#define MPU_SMPLRT_DIV         0x19   // R/W
#define MPU_CONFIG             0x1A   // R/W
#define MPU_GYRO_CONFIG        0x1B   // R/W
#define MPU_ACCEL_CONFIG       0x1C   // R/W
#define MPU_FF_THR             0x1D   // R/W
#define MPU_FF_DUR             0x1E   // R/W
#define MPU_MOT_THR            0x1F   // R/W
#define MPU_MOT_DUR            0x20   // R/W
#define MPU_ZRMOT_THR          0x21   // R/W
#define MPU_ZRMOT_DUR          0x22   // R/W
#define MPU_FIFO_EN            0x23   // R/W
#define MPU_I2C_MST_CTRL       0x24   // R/W
#define MPU_I2C_SLV0_ADDR      0x25   // R/W
#define MPU_I2C_SLV0_REG       0x26   // R/W
#define MPU_I2C_SLV0_CTRL      0x27   // R/W
#define MPU_I2C_SLV1_ADDR      0x28   // R/W
#define MPU_I2C_SLV1_REG       0x29   // R/W
#define MPU_I2C_SLV1_CTRL      0x2A   // R/W
#define MPU_I2C_SLV2_ADDR      0x2B   // R/W
#define MPU_I2C_SLV2_REG       0x2C   // R/W
#define MPU_I2C_SLV2_CTRL      0x2D   // R/W
#define MPU_I2C_SLV3_ADDR      0x2E   // R/W
#define MPU_I2C_SLV3_REG       0x2F   // R/W
#define MPU_I2C_SLV3_CTRL      0x30   // R/W
#define MPU_I2C_SLV4_ADDR      0x31   // R/W
#define MPU_I2C_SLV4_REG       0x32   // R/W
#define MPU_I2C_SLV4_DO        0x33   // R/W
#define MPU_I2C_SLV4_CTRL      0x34   // R/W
#define MPU_I2C_SLV4_DI        0x35   // R  
#define MPU_I2C_MST_STATUS     0x36   // R
#define MPU_INT_PIN_CFG        0x37   // R/W
#define MPU_INT_ENABLE         0x38   // R/W
#define MPU_INT_STATUS         0x3A   // R  
#define MPU_ACCEL_XOUT_H       0x3B   // R  
#define MPU_ACCEL_XOUT_L       0x3C   // R  
#define MPU_ACCEL_YOUT_H       0x3D   // R  
#define MPU_ACCEL_YOUT_L       0x3E   // R  
#define MPU_ACCEL_ZOUT_H       0x3F   // R  
#define MPU_ACCEL_ZOUT_L       0x40   // R  
#define MPU_TEMP_OUT_H         0x41   // R  
#define MPU_TEMP_OUT_L         0x42   // R  
#define MPU_GYRO_XOUT_H        0x43   // R  
#define MPU_GYRO_XOUT_L        0x44   // R  
#define MPU_GYRO_YOUT_H        0x45   // R  
#define MPU_GYRO_YOUT_L        0x46   // R  
#define MPU_GYRO_ZOUT_H        0x47   // R  
#define MPU_GYRO_ZOUT_L        0x48   // R  
#define MPU_EXT_SENS_DATA_00   0x49   // R  
#define MPU_EXT_SENS_DATA_01   0x4A   // R  
#define MPU_EXT_SENS_DATA_02   0x4B   // R  
#define MPU_EXT_SENS_DATA_03   0x4C   // R  
#define MPU_EXT_SENS_DATA_04   0x4D   // R  
#define MPU_EXT_SENS_DATA_05   0x4E   // R  
#define MPU_EXT_SENS_DATA_06   0x4F   // R  
#define MPU_EXT_SENS_DATA_07   0x50   // R  
#define MPU_EXT_SENS_DATA_08   0x51   // R  
#define MPU_EXT_SENS_DATA_09   0x52   // R  
#define MPU_EXT_SENS_DATA_10   0x53   // R  
#define MPU_EXT_SENS_DATA_11   0x54   // R  
#define MPU_EXT_SENS_DATA_12   0x55   // R  
#define MPU_EXT_SENS_DATA_13   0x56   // R  
#define MPU_EXT_SENS_DATA_14   0x57   // R  
#define MPU_EXT_SENS_DATA_15   0x58   // R  
#define MPU_EXT_SENS_DATA_16   0x59   // R  
#define MPU_EXT_SENS_DATA_17   0x5A   // R  
#define MPU_EXT_SENS_DATA_18   0x5B   // R  
#define MPU_EXT_SENS_DATA_19   0x5C   // R  
#define MPU_EXT_SENS_DATA_20   0x5D   // R  
#define MPU_EXT_SENS_DATA_21   0x5E   // R  
#define MPU_EXT_SENS_DATA_22   0x5F   // R  
#define MPU_EXT_SENS_DATA_23   0x60   // R  
#define MPU_MOT_DETECT_STATUS  0x61   // R  
#define MPU_I2C_SLV0_DO        0x63   // R/W
#define MPU_I2C_SLV1_DO        0x64   // R/W
#define MPU_I2C_SLV2_DO        0x65   // R/W
#define MPU_I2C_SLV3_DO        0x66   // R/W
#define MPU_I2C_MST_DELAY_CTRL 0x67   // R/W
#define MPU_SIGNAL_PATH_RESET  0x68   // R/W
#define MPU_MOT_DETECT_CTRL    0x69   // R/W
#define MPU_USER_CTRL          0x6A   // R/W
#define MPU_PWR_MGMT_1         0x6B   // R/W
#define MPU_PWR_MGMT_2         0x6C   // R/W
#define MPU_FIFO_COUNTH        0x72   // R/W
#define MPU_FIFO_COUNTL        0x73   // R/W
#define MPU_FIFO_R_W           0x74   // R/W
#define MPU_WHO_AM_I           0x75   // R

//MPU Compass (only on MPU9150)
#define MPU_CMPS_XOUT_L        0x4A   // R
#define MPU_CMPS_XOUT_H        0x4B   // R
#define MPU_CMPS_YOUT_L        0x4C   // R
#define MPU_CMPS_YOUT_H        0x4D   // R
#define MPU_CMPS_ZOUT_L        0x4E   // R
#define MPU_CMPS_ZOUT_H        0x4F   // R


// Number of MPU sensors to address
#ifndef MPU_count
#define MPU_count 2
#endif

// The sensor's I2C address is 0x69 (AD0==1 or unconnected) or 0x68 (AD0=0)
#define MPU_I2C_ADDRESS  0x69 
int MPU_I2C_TARGET=MPU_I2C_ADDRESS;

#ifndef MPU_HAS_SELECT_PINS
// Daniel Kling's breakout board:
const unsigned char MPU_selectPins[6]={ 11,10,9,8,A0,12 };
#endif

/*
 Pull this sensor's select pin high
*/
void MPU_selectSensor(int sensor)
{
    digitalWrite(MPU_selectPins[sensor],HIGH);
}
void MPU_deselectSensor(int sensor)
{
    digitalWrite(MPU_selectPins[sensor],LOW);
}

/// Store a 16-bit int with the *high* byte first in memory
struct int16_big_endian {
  int8_t hi; // high byte (and sign bit)
  uint8_t lo; // low byte
  
  void zero(void) { lo=hi=0; }
  
  // Allow a normal int to get extracted from this class,
  //   so things like "int x=r.accel.x;" just work.
  operator int () const {
    return (hi<<8)|lo;
  }
};

/// Store a 3D vector of 16-bit ints
struct vec3_int16 {
  int16_big_endian x,y,z; // value along different axes
  
  void zero(void) {
    x.zero(); y.zero(); z.zero();
  }

  void print(const char *name) {
#if _STDIO_H

#else // arduino?
    Serial.print(name);
    Serial.print("=(");
    Serial.print(x); Serial.print(" ");
    Serial.print(y); Serial.print(" ");
    Serial.print(z); 
    Serial.print(") ");
#endif
  }
  void printAcc(const char *name) {
    Serial.print(name);
    Serial.print("=(");
    Serial.print(x>>6); Serial.print(" ");
    Serial.print(y>>6); Serial.print(" ");
    Serial.print(z>>6); 
    Serial.print(") ");
  }
  void printGyro(const char *name) {
    Serial.print(name);
    Serial.print("=(");
    Serial.print(x>>4); Serial.print(" ");
    Serial.print(y>>4); Serial.print(" ");
    Serial.print(z>>4); 
    Serial.print(") ");
  }
  
  // Quantize our value into this vector type, shifting away this many bits
  //  Return true if the value is too bit to fit.
  bool quantize(nanoslot_xyz10_t &v,unsigned char shift) {
    bool too_big=false;
    v.x=quantize(x,shift,too_big);
    v.y=quantize(y,shift,too_big);
    v.z=quantize(z,shift,too_big);
    return too_big;
  }
private:
  // Quantize src>>shift to a signed 10-bit value.  
  //  Set too_big if it doesn't fit.
  int quantize(int src,unsigned char shift,bool &too_big) 
  {
    int i=src>>shift;
    if (i>510 || i<-510) {
        too_big=true;
        return -512;
    }
    else {
        return i;
    }
  }
};

// Binary struct stores the raw data sent by the IMU:
struct IMU_report {
  vec3_int16 accel;
  int16_big_endian temp;
  vec3_int16 gyro;
  
  void zero() {
    accel.zero();
    temp.zero();
    gyro.zero();
  }
  
  // Return true if we have valid data
  bool valid() {
    return temp>0 || temp<-1;
  }
  
  // Convert our data to an IMU struct:
  //   Throw away noisy low bits as we figure out the scale factor.
  void to_IMU(nanoslot_IMU_t &imu)
  {
    imu.acc.type=0;
    while (accel.quantize(imu.acc,6+imu.acc.type) && imu.acc.type<3) imu.acc.type++;
    imu.gyro.type=0;
    while (gyro.quantize(imu.gyro,4+imu.gyro.type) && imu.gyro.type<3) imu.gyro.type++;
  }

  // Return the temperature in degrees C
  int get_temperature() {
    return (((int)temp) + 521)/340 + 35;
  }

  // Print our data to the serial port
  void print()
  {
    accel.printAcc(" accel");
    gyro.printGyro("gyro");
    Serial.print(" t ");
    Serial.print(get_temperature());
  }

  void read(int sensor) {
    MPU_selectSensor(sensor);
    
    // Request data from the IMU:
    Wire.beginTransmission(MPU_I2C_TARGET);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);

    // Read the bytes of the IMU report:
    IMU_report ret;
    Wire.requestFrom(MPU_I2C_TARGET,sizeof(ret),true);
    temp.zero();
    
    char *buffer=(char *)&accel; // write into us as bytes
    for (int i=0;i<sizeof(*this);i++) buffer[i]=Wire.read();
    
    MPU_deselectSensor(sensor);
    
    return ret;
  }
};

// Send this one-byte data value to this address.
int MPU_writeSensor(int addr,int data){
  Wire.beginTransmission(MPU_I2C_TARGET);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission(true);

  return 1;
}


/*
 Configure this one sensor.  Call once at startup.
http://pansenti.wordpress.com/2013/03/26/pansentis-invensense-mpu-9150-software-for-arduino-is-now-on-github/
*/
void MPU_setup_single(int sensor){
  MPU_selectSensor(sensor);
  
// Clear the 'sleep' bit to start the sensor.
  MPU_writeSensor(MPU_PWR_MGMT_1, 0);

//  MPU_writeSensor(MPU_GYRO_CONFIG, 0x18); //2000 deg/sec full scale
  MPU_writeSensor(MPU_GYRO_CONFIG, 0x0); //250 deg/sec best accuracy
  MPU_writeSensor(MPU_ACCEL_CONFIG, 0x0); // +- 2 g range
  
//  MPU_writeSensor(MPU_CONFIG, 0x4); //20Hz low pass filter (DLPF)
  MPU_writeSensor(MPU_CONFIG, 0x3); //42Hz low pass filter (DLPF)

  MPU_deselectSensor(sensor);
}



/* External interface: set up the sensors.  Call once at startup. */
void MPU_setup()
{
  Wire.begin(); //<- super easy to forget!
  Wire.setWireTimeout(5000,true);
  
  for (int sensor=0;sensor<MPU_count;sensor++) {
    pinMode(MPU_selectPins[sensor],OUTPUT);
    digitalWrite(MPU_selectPins[sensor],LOW);
  }
  
  for (int sensor=0;sensor<MPU_count;sensor++)
  {
    MPU_setup_single(sensor);
  }
}

/* External interface: Read data from this sensor, and convert to an imu report. */
IMU_report MPU_lastReport;
bool MPU_read(int sensor,nanoslot_IMU_t &imu)
{
  MPU_lastReport.read(sensor);
  if (MPU_lastReport.valid())
  {
      MPU_lastReport.to_IMU(imu);
      return true;
  } else {
      imu.invalidate();
      return false;
  }
}



#endif
