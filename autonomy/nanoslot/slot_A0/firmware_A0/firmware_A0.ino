
#define NANOSLOT_MY_ID 0xA0 /* robot Arm board 0 */
#include "nanoslot/firmware.h"

milli_t last_time=0;
milli_t worst_latency=0;
void firmware_read_encoders(void)
{
  milli_t now=milli;
  milli_t latency=now-last_time;
  last_time=now;  
  if (latency>worst_latency) worst_latency=latency;
  my_sensor.latency=worst_latency;
  my_sensor.heartbeat++;
}

bool firmware_handle_custom_packet(A_packet_serial &pkt,A_packet &p)
{
  return false;
}

void firmware_send_motors()
{
  if (!comm.is_connected) my_command.mode=0;

  
}


void setup() {
    nanoslot_firmware_start();
}

void loop() {
    nanoslot_firmware_loop();
}
