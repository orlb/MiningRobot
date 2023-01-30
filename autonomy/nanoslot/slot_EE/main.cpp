/*
 Example where we interface the data exchange with slot serial comms.

 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-01-25 (Public Domain)
*/
#define NANOSLOT_MY_ID 0xEE /* my numeric slot ID */
#define SLOT nano.slot_EE  /* my exchange struct */
#include "aurora/lunatic.h"
#include "nanoslot/nanoboot_handoff.h"

int main(int argc,char **argv)
{
    nanoslot_comms comm(&argc,&argv);
    MAKE_exchange_nanoslot(); 
    exchange_nanoslot.write_begin().sanity_check_size();
    exchange_nanoslot.write_end();
    
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
                
                printf("  Arduino latency: %d ms, heartbeat %02x\n", my_sensor.latency, my_sensor.heartbeat);
            }
            
            if (comm.need_command) 
            {
                const nanoslot_exchange &nano=exchange_nanoslot.read();
                NANOSLOT_COMMAND_MY cmd=SLOT.command;
                cmd.autonomy=nano.autonomy;
                comm.send_command(cmd);
            }
        }
        
        // Limit this loop speed to this many milliseconds (varies by what's attached)
        data_exchange_sleep(50);
    }
    
    return 0;
}

