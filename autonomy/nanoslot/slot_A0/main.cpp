/*
 Interface the lunatic data exchange with slot A0 arm nano.

 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-01-25 (Public Domain)
*/
#define NANOSLOT_MY_ID 0xA0 /* my numeric slot ID */
#define NANOSLOT_MY_EX nano.slot_A0  /* my exchange struct */
#include "aurora/lunatic.h"
#include "nanoslot/nanoboot_handoff.h"

int main(int argc,char **argv)
{
    nanoslot_lunatic comm(&argc,&argv);
    
    while (comm.is_connected) {
        // Receive data from Arduino
        A_packet p;
        if (comm.read_packet(p)) {
            comm.handle_standard_packet(p);

            if (comm.got_sensor) 
            {
                comm.my_sensor.imu0.acc.print("  acc0");
                comm.my_sensor.imu0.gyro.print("  gyro0");
                comm.my_sensor.imu1.acc.print("  acc1");
                comm.my_sensor.imu1.gyro.print("  gyro1");
                printf("\n");
                fflush(stdout);
                
                if (comm.verbose) 
                {
                    printf("  A0 feedback: %02x\n",comm.my_sensor.feedback); fflush(stdout);
                }
            }
            
            if (comm.need_command)
            {
                comm.send_command(comm.my_command);
                if (comm.verbose) {
                    printf("  A0 motors: %3d %3d\n",comm.my_command.motor[0],comm.my_command.motor[1]); fflush(stdout);
                }
            }
        }
        
        // Limit this loop speed to this many milliseconds (varies by what's attached)
        data_exchange_sleep(50);
    }
    
    return 0;
}

