/*
 Interface the lunatic data exchange with slot A1 arm nano.

 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-01-25 (Public Domain)
*/
#define NANOSLOT_MY_ID 0xF1 /* my numeric slot ID */
#define NANOSLOT_MY_EX nano.slot_F1  /* my exchange struct */
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
                printf("   F1: ");
                for (int i=0;i<NANOSLOT_SENSOR_MY::n_imu;i++)
                {
                    comm.my_sensor.imu[i].acc.print("  acc ");
                    comm.my_sensor.imu[i].gyro.print(" gyro ");
                }
                printf("\n");
                fflush(stdout);
            }
            
            if (comm.need_command)
            {
                comm.send_command(comm.my_command);
            }
        }
        
        // Limit this loop speed to this many milliseconds (varies by what's attached)
        data_exchange_sleep(50);
    }
    
    return 0;
}

