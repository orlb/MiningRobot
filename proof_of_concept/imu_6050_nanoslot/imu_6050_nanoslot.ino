#define MPU_COUNT 3
#include "nanoslot/nanoslot_exchange.h"
#include "nanoslot/firmware_mpu6050.h"

void setup()
{
  MPU_setup();
  Serial.begin(115200);
  Serial.println("MPU test via nanoslot firmware header");
}
void loop() 
{
  nanoslot_IMU_t imu0, imu1, imu2;
  if (MPU_read(0,imu0))
  {
    Serial.print("    imu0 ");
    Serial.print(imu0.acc.x);
    if (imu0.acc.type>0) {
      Serial.print("scale");
      Serial.print(imu0.acc.type);
    }
  }
  
  if (MPU_read(1,imu1)) {
    Serial.print("    imu1 ");
    Serial.println(imu1.acc.x);
  }

  if (MPU_read(2,imu1)) {
    Serial.print("    imu2 ");
    Serial.println(imu2.acc.x);
  }

  delay(100);
}
