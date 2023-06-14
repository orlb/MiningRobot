/*
 Firmware that runs on robot rockgrinder mining head
*/
#define NANOSLOT_MY_ID 0xC0
#include "nanoslot/firmware.h"
#include <Servo.h>

const int minePin=3; // mining speed controller PWM pin
Servo mine;
const int pwmStop=1000; // microsecond RC PWM width for stop
const int pwmFull=1000; // microsecond RC PWM difference for full power

const int spinPin=8; 
const int spinPinVcc=9; 
const int spinPinGnd=10; 
int spinLast=0;

void firmware_read_encoders(void)
{
  my_sensor.heartbeat++;
  
  int spinCur=digitalRead(spinPin);
  if (spinCur!=spinLast) {
      my_sensor.spincount++;
      spinLast = spinCur;
  }

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
  pinMode(13,OUTPUT); // blink pin
  
  pinMode(spinPin,INPUT_PULLUP); // soft pull-up on spin encoder pin
  pinMode(spinPinVcc,OUTPUT); digitalWrite(spinPinVcc,HIGH); // soft power for encoder
  pinMode(spinPinGnd,OUTPUT); digitalWrite(spinPinGnd,LOW);
  
  mine.attach(minePin); // motor controller command pin
  nanoslot_firmware_start();
}

void loop() {
  nanoslot_firmware_loop();
}
