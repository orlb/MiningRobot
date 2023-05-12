/*
 Firmware that runs on robot mining head
*/
#define NANOSLOT_MY_ID 0xC0
#include "nanoslot/firmware.h"
#include <Servo.h>

Servo mine;
const int pwmStop=1000;
const int pwmFull=1000; 

void firmware_read_encoders(void)
{
  my_sensor.heartbeat++;
  my_sensor.spin = my_command.mine;
  my_sensor.cell0=analogRead(A7); // JST gnd pin
  my_sensor.cell1=analogRead(A6); // JST first lipo cell
}

void firmware_send_motors()
{
  if (!comm.is_connected) my_command.autonomy.mode=0;
  
  int cmd=pwmStop;
  if (my_command.autonomy.mode!=0) {
    cmd = pwmStop + pwmFull*(long)my_command.mine/100;
  }
  mine.writeMicroseconds(cmd);
  digitalWrite(13,(cmd>pwmStop+100)?1:0);
}


bool firmware_handle_custom_packet(A_packet_serial &pkt,A_packet &p)
{
  return false;
}

void setup() {
  pinMode(13,OUTPUT);
  mine.attach(3); // motor controller command pin hardwired to pin 3
  nanoslot_firmware_start();
}

void loop() {
  nanoslot_firmware_loop();
}
