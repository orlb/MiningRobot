/*
 Firmware that runs on robot forward
*/
#define NANOSLOT_MY_ID 0xD0
#include "nanoslot/firmware.h"

enum {n_encoder=2};
int encoder_pins[n_encoder] = {A2,A3}; // left and right motor encoder pins
int encoder_counts[n_encoder] = {0,0};
int encoder_last[n_encoder] = {0,0};

#include "nanoslot/firmware_un178.h"

void firmware_read_encoders(void)
{
  my_sensor.heartbeat++;
  //my_sensor.stop=0; // DISABLED: (digitalRead(ESTOP_pin)!=0);
  
  for (int i=0;i<n_encoder;i++) {
    int cur=digitalRead(encoder_pins[i]);
    if (cur!=encoder_last[i]) {
        encoder_counts[i]++;
        encoder_last[i]=cur;
    }
    my_sensor.counts[i]=encoder_counts[i];
  }
  
}

void firmware_send_motors()
{
  if (!comm.is_connected) my_command.autonomy.mode=0;
  
  NANOSLOT_MOTOR_SEND_POWER();
}


bool firmware_handle_custom_packet(A_packet_serial &pkt,A_packet &p)
{
  return false;
}

void setup() {
  for (int p:encoder_pins) pinMode(p,INPUT_PULLUP);
  NANOSLOT_MOTOR_SETUP();
  nanoslot_firmware_start();
}

void loop() {
  nanoslot_firmware_loop();
}
