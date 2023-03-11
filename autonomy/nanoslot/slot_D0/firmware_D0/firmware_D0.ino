/*
 Firmware that runs on robot forward
*/
#define NANOSLOT_MY_ID 0xD0
#include "nanoslot/firmware.h"

#include "nanoslot/firmware_un178.h"

void firmware_read_encoders(void)
{
  my_sensor.heartbeat++;
  my_sensor.stop=0; // DISABLED: (digitalRead(ESTOP_pin)!=0);
}

void firmware_send_motors()
{
  if (!comm.is_connected || my_sensor.stop) my_command.autonomy.mode=0;
  
  NANOSLOT_MOTOR_SEND_POWER();
}


bool firmware_handle_custom_packet(A_packet_serial &pkt,A_packet &p)
{
  return false;
}

void setup() {
  NANOSLOT_MOTOR_SETUP();
  nanoslot_firmware_start();
}

void loop() {
  nanoslot_firmware_loop();
}
