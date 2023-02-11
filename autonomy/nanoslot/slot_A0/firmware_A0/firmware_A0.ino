/*
 Firmware that runs on robot arm board 0
*/
#define NANOSLOT_MY_ID 0xA0 /* robot Arm board 0 */
#include "nanoslot/firmware.h"

#define NeoPIN        A1 /* neopixel chain is on a 3-pin header */
#include "nanoslot/firmware_neopixel.h"

const int ESTOP_pin=A2; /* ESTOP switch: ground=run mode, open=stop */

/* accelerometer/gyro IMUs:
   0 is on tool coupler
   1 is on stick frame
   (future) 2 is on boom
*/
#define MPU_COUNT 2 
#include "nanoslot/firmware_mpu6050.h"

#include "nanoslot/firmware_un178.h"
/* Hardware-connected motor drivers, UN178 green brushed boards */
un178_motor_single_t hardware_motor[NANOSLOT_COMMAND_MY::n_motors]={
    //un178_motor_single_t(11,12,A0),
    //un178_motor_single_t(10,9,8),
    un178_motor_single_t(3,2,4),
    un178_motor_single_t(6,5,7),
  };

void firmware_read_encoders(void)
{
  MPU_read(0,my_sensor.imu0);
  MPU_read(1,my_sensor.imu1);
  //MPU_read(2,my_sensor.imu2);

  my_sensor.heartbeat++;
  my_sensor.stop=(digitalRead(ESTOP_pin)!=0);
  my_sensor.feedback=MPU_lastReport.accel.x.hi;
}

void firmware_send_motors()
{
  if (!comm.is_connected || my_sensor.stop) my_command.autonomy.mode=0;
  
  NANOSLOT_MOTOR_SEND_POWER();

  updateNeopixels(my_command.autonomy.mode);
}


bool firmware_handle_custom_packet(A_packet_serial &pkt,A_packet &p)
{
  return false;
}

void setup() {
  pinMode(ESTOP_pin,INPUT_PULLUP);
  NANOSLOT_MOTOR_SETUP();
  neopixels.begin();
  nanoslot_firmware_start();
  MPU_setup();
}

void loop() {
  nanoslot_firmware_loop();
}
