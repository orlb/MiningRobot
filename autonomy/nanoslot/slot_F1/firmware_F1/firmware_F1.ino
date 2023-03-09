/*
 Firmware that runs on robot arm board F1 up front
*/
#define NANOSLOT_MY_ID 0xF1
#include "nanoslot/firmware.h"

/* accelerometer/gyro IMUs:
   0 is on tool coupler
   1 is on stick frame
*/
#define MPU_COUNT (NANOSLOT_SENSOR_MY::n_imu)  
#include "nanoslot/firmware_mpu6050.h"

void firmware_read_encoders(void)
{
  for (int i=0;i<MPU_COUNT;i++)
      MPU_read(i,my_sensor.imu[i]);

  my_sensor.heartbeat++;
}

void firmware_send_motors()
{
  if (!comm.is_connected) my_command.autonomy.mode=0;
}


bool firmware_handle_custom_packet(A_packet_serial &pkt,A_packet &p)
{
  return false;
}

void setup() {
  MPU_setup();
  nanoslot_firmware_start();
}

void loop() {
  nanoslot_firmware_loop();
}
