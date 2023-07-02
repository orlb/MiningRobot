/*
 Firmware that runs on robot forward
*/
#define NANOSLOT_MY_ID 0xF0
#include "nanoslot/firmware.h"

// const int ESTOP_pin=A1; /* ESTOP switch: ground=run mode, open=stop */

#include "nanoslot/firmware_un178.h"

void firmware_read_encoders(void)
{
  my_sensor.heartbeat++;
  my_sensor.stop=0; // DISABLED: (digitalRead(ESTOP_pin)!=0);
  
  my_sensor.cell1=analogRead(A1); // JST first lipo cell
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
  //pinMode(ESTOP_pin,INPUT_PULLUP);
  NANOSLOT_MOTOR_SETUP();
  nanoslot_firmware_start();
}

void loop() {
  nanoslot_firmware_loop();
}
