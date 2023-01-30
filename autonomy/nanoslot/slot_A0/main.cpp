/*
 Interface the lunatic data exchange with slot A0 arm nano.

 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-01-25 (Public Domain)
*/
#define NANOSLOT_MY_ID 0xA0 /* my numeric slot ID */
#define SLOT nano.slot_A0  /* my exchange struct */
#include "aurora/lunatic.h"
#include "nanoslot/nanoboot_handoff.h"

int main(int argc,char **argv)
{
    nanoslot_comms comm(&argc,&argv);
    NANOSLOT_EXCHANGE_SETUP();
    NANOSLOT_SENSOR_MY my_sensor={0};
    
    while (comm.is_connected) {
        // Receive data from Arduino
        A_packet p;
        if (comm.read_packet(p)) {
            comm.handle_standard_packet(p,my_sensor);

            if (comm.got_sensor) 
            {
                nanoslot_exchange &nano=exchange_nanoslot.write_begin();
                SLOT.sensor=my_sensor;
                SLOT.debug.packet_count++;
                exchange_nanoslot.write_end();
                if (comm.verbose) {
                    printf("  A0 feedback: %02x\n",my_sensor.feedback); fflush(stdout);
                }
            }
            
            if (comm.need_command)
            {
                const nanoslot_exchange &nano=exchange_nanoslot.read();
                NANOSLOT_EXCHANGE_COMMAND();
                comm.send_command(my_command);
                if (comm.verbose) {
                    printf("  A0 motors: %3d %3d\n",my_command.motor[0],my_command.motor[1]); fflush(stdout);
                }
            }
        }
        
        // Limit this loop speed to this many milliseconds (varies by what's attached)
        data_exchange_sleep(50);
    }
    
    return 0;
}

